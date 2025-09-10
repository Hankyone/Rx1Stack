// rx1tui — Bubble Tea TUI to scan and control Feetech serial‑bus servos via URT‑1
//
// Build:
//   go mod init rx1tui
//   go get github.com/charmbracelet/bubbletea@latest \
//          github.com/charmbracelet/bubbles@latest \
//          github.com/charmbracelet/lipgloss@latest \
//          github.com/tarm/serial@latest
//   go build -o rx1tui
//
// Run:
//   ./rx1tui -port /dev/tty.usbserial-XXXXX -baud 1000000 -family sts
//
// Keys:
//   r     — rescan bus
//   enter — toggle torque on selected servo
//   p     — set position (degrees for STS family)
//   u     — set position (raw units)
//   i     — change ID (writes register 5; will unlock/lock EEPROM)
//   l     — edit soft limits (min/max, raw units)
//   d     — dump quick telemetry (regs)
//   q     — quit
//
// Notes:
//  • Defaults target Feetech STS/SMS ("sts") protocol mapping; SCS should mostly work
//    but register addresses may differ subtly. Adjust constants below if needed.
//  • Degrees→units uses 4096/360 mapping (STS3215: 0–360° absolute, 4096 ticks). If your
//    model differs, tweak unitsPerDeg.
//  • Minimal error handling; intended as a bring‑up/bench tool and LLM‑friendly base.
//
package main

import (
    "bufio"
    "bytes"
    "encoding/hex"
    "errors"
    "flag"
    "fmt"
    "log"
    "math"
    "os"
    "sort"
    "strconv"
    "strings"
    "time"

    tea "github.com/charmbracelet/bubbletea"
    "github.com/charmbracelet/bubbles/table"
    "github.com/charmbracelet/bubbles/textinput"
    "github.com/charmbracelet/lipgloss"
    serial "github.com/tarm/serial"
)

// -------------------------- Feetech protocol (minimal) --------------------------
// Packet:
//  Tx: 0xFF 0xFF ID LEN INST PARAM... CHKSUM ; where CHKSUM = ^(ID+LEN+INST+∑PARAM) & 0xFF
//  Rx: 0xFF 0xFF ID LEN ERR  DATA...  CHKSUM
// Instructions
const (
    instPing   = 0x01
    instRead   = 0x02
    instWrite  = 0x03
    instRWrite = 0x04
    instAction = 0x05
    // 0x83 sync write not used here
)

// Registers (STS family; decimal addresses to match common docs)
const (
    regModel           = 3  // 2 bytes
    regID              = 5  // 1 byte
    regBaud            = 6  // 1 byte (0 -> 1,000,000)
    regAngleMin        = 9  // 2 bytes
    regAngleMax        = 11 // 2 bytes
    regLock            = 55 // 1 byte (0 unlock, 1 lock)
    regTorqueEnable    = 40 // 1 byte (0 off, 1 on)
    regGoalPosition    = 42 // 2 bytes (low, high)
    regGoalTime        = 44 // 2 bytes (ms)  
    regGoalSpeed       = 46 // 2 bytes
    regPresentPosition = 56 // 2 bytes
    regPresentSpeed    = 58 // 2 bytes
    regPresentVoltage  = 62 // 1 byte (x10 -> Volts)
    regPresentTemp     = 63 // 1 byte (°C)
)

// unitsPerDeg converts degrees to encoder units for STS3215 (4096/360 ≈ 11.3777)
const unitsPerDeg = 4096.0 / 360.0

// feetechBus wraps a tarm/serial port
type feetechBus struct {
    port   *serial.Port
    reader *bufio.Reader
}

func openBus(dev string, baud int) (*feetechBus, error) {
    cfg := &serial.Config{Name: dev, Baud: baud, ReadTimeout: time.Millisecond * 30}
    p, err := serial.OpenPort(cfg)
    if err != nil { return nil, err }
    return &feetechBus{port: p, reader: bufio.NewReader(p)}, nil
}

func (b *feetechBus) close() { if b.port != nil { _ = b.port.Close() } }

func checksum(bytesSum int) byte { return byte(^bytesSum & 0xFF) }

func (b *feetechBus) tx(id byte, inst byte, params ...byte) error {
    length := 2 + len(params) // INST + PARAMS + CHKSUM? (LEN includes INST+PARAMS+CHK per manual uses N+2)
    sum := int(id) + length + int(inst)
    pkt := []byte{0xFF, 0xFF, id, byte(length), inst}
    if len(params) > 0 {
        pkt = append(pkt, params...)
        for _, p := range params { sum += int(p) }
    }
    pkt = append(pkt, checksum(sum))
    _, err := b.port.Write(pkt)
    return err
}

// readReply reads a single reply, returns (errCode, data, error)
func (b *feetechBus) readReply() (byte, []byte, error) {
    // find header 0xFF 0xFF
    // naive sync: read until we see 0xFF 0xFF
    var buf [1]byte
    for {
        b.port.Read(buf[:])
        if buf[0] == 0xFF {
            b.port.Read(buf[:])
            if buf[0] == 0xFF { break }
        }
    }
    // ID
    id, err := b.reader.ReadByte(); if err != nil { return 0, nil, err }
    // LEN
    ln, err := b.reader.ReadByte(); if err != nil { return 0, nil, err }
    // ERR
    errb, err := b.reader.ReadByte(); if err != nil { return 0, nil, err }
    // DATA length = ln - 2 (ERR + CHK)
    dataLen := int(ln) - 2
    if dataLen < 0 { dataLen = 0 }
    data := make([]byte, dataLen)
    if dataLen > 0 {
        if _, err := b.reader.Read(data); err != nil { return 0, nil, err }
    }
    // checksum
    cks, err := b.reader.ReadByte(); if err != nil { return 0, nil, err }
    // verify checksum (optional)
    sum := int(id) + int(ln) + int(errb)
    for _, d := range data { sum += int(d) }
    if checksum(sum) != cks {
        return errb, data, fmt.Errorf("checksum mismatch: got %02X want %02X", cks, checksum(sum))
    }
    return errb, data, nil
}

func (b *feetechBus) readRegs(id byte, addr byte, n byte) ([]byte, error) {
    if err := b.tx(id, instRead, addr, n); err != nil { return nil, err }
    errb, data, err := b.readReply()
    if err != nil { return nil, err }
    if errb != 0 { return nil, fmt.Errorf("servo err=0x%02X", errb) }
    if len(data) != int(n) { return nil, fmt.Errorf("short read: %d != %d", len(data), n) }
    return data, nil
}

func (b *feetechBus) writeRegs(id byte, addr byte, payload ...byte) error {
    params := append([]byte{addr}, payload...)
    if err := b.tx(id, instWrite, params...); err != nil { return err }
    errb, _, err := b.readReply()
    if err != nil { return err }
    if errb != 0 { return fmt.Errorf("servo err=0x%02X", errb) }
    return nil
}

// helpers
func le16(lo, hi byte) int { return int(lo) | (int(hi) << 8) }
func toLE16(v int) (byte, byte) { return byte(v & 0xFF), byte((v>>8) & 0xFF) }

// ------------------------------ App model --------------------------------------

type Servo struct {
    ID      int
    Model   int
    Pos     int
    Volt10  int // voltage x10
    TempC   int
    Torque  bool
    Min     int
    Max     int
}

type appModel struct {
    port    string
    baud    int
    family  string
    bus     *feetechBus

    table   table.Model
    rows    []table.Row
    servos  []Servo

    status  string
    err     error

    entering bool
    prompt   string
    input    textinput.Model
    action   string // "pos_deg", "pos_units", "set_id", "limits"
}

type tickMsg struct{}
type rescanMsg struct{ servos []Servo; err error }

type writeDoneMsg struct{ err error }

type dumpMsg struct{ text string; err error }

func newApp(port string, baud int, family string) appModel {
    ti := textinput.New()
    ti.CharLimit = 16
    ti.Prompt = "> "

    columns := []table.Column{
        {Title: "ID", Width: 5},
        {Title: "Model", Width: 6},
        {Title: "Pos", Width: 6},
        {Title: "Volt(V)", Width: 8},
        {Title: "Temp(°C)", Width: 8},
        {Title: "Torque", Width: 7},
        {Title: "Min", Width: 6},
        {Title: "Max", Width: 6},
    }
    t := table.New(table.WithColumns(columns), table.WithHeight(12))
    t.SetStyles(defaultTableStyles())

    return appModel{port: port, baud: baud, family: family, table: t, input: ti}
}

func (m appModel) Init() tea.Cmd {
    return tea.Batch(m.openBus(), rescan(m.port, m.baud))
}

func (m *appModel) openBus() tea.Cmd {
    return func() tea.Msg {
        bus, err := openBus(m.port, m.baud)
        if err != nil { m.err = err; m.status = err.Error(); return tickMsg{} }
        m.bus = bus; m.status = fmt.Sprintf("open %s @ %d", m.port, m.baud)
        return tickMsg{}
    }
}

func rescan(port string, baud int) tea.Cmd {
    return func() tea.Msg {
        bus, err := openBus(port, baud)
        if err != nil { return rescanMsg{nil, err} }
        defer bus.close()
        // scan IDs 1..200 by reading 2 bytes at present position
        ids := []int{}
        for id := 1; id <= 200; id++ {
            if _, err := bus.readRegs(byte(id), regPresentPosition, 2); err == nil {
                ids = append(ids, id)
            }
        }
        servos := make([]Servo, 0, len(ids))
        for _, id := range ids {
            // gather basic regs
            modelb, _ := bus.readRegs(byte(id), regModel, 2)
            posb, _ := bus.readRegs(byte(id), regPresentPosition, 2)
            volb, _ := bus.readRegs(byte(id), regPresentVoltage, 1)
            tmpb, _ := bus.readRegs(byte(id), regPresentTemp, 1)
            tqb, _ := bus.readRegs(byte(id), regTorqueEnable, 1)
            minb, _ := bus.readRegs(byte(id), regAngleMin, 2)
            maxb, _ := bus.readRegs(byte(id), regAngleMax, 2)
            s := Servo{
                ID: id,
                Model: le16(modelb[0], modelb[1]),
                Pos: le16(posb[0], posb[1]),
                Volt10: int(volb[0]),
                TempC: int(tmpb[0]),
                Torque: tqb[0] != 0,
                Min: le16(minb[0], minb[1]),
                Max: le16(maxb[0], maxb[1]),
            }
            servos = append(servos, s)
        }
        sort.Slice(servos, func(i, j int) bool { return servos[i].ID < servos[j].ID })
        return rescanMsg{servos: servos}
    }
}

func (m appModel) Update(msg tea.Msg) (tea.Model, tea.Cmd) {
    switch msg := msg.(type) {
    case tea.KeyMsg:
        if m.entering {
            switch msg.Type {
            case tea.KeyEnter:
                val := strings.TrimSpace(m.input.Value())
                m.entering = false
                m.status = ""
                switch m.action {
                case "pos_deg":
                    id := m.selectedID()
                    deg, err := strconv.ParseFloat(val, 64)
                    if err != nil { m.err = err; return m, nil }
                    units := int(math.Round(deg * unitsPerDeg))
                    return m, m.writeGoalPos(id, units)
                case "pos_units":
                    id := m.selectedID()
                    units, err := strconv.Atoi(val)
                    if err != nil { m.err = err; return m, nil }
                    return m, m.writeGoalPos(id, units)
                case "set_id":
                    id := m.selectedID()
                    newID, err := strconv.Atoi(val)
                    if err != nil { m.err = err; return m, nil }
                    return m, m.changeID(id, newID)
                case "limits":
                    parts := strings.Fields(val)
                    if len(parts) != 2 { m.err = errors.New("enter: <min> <max>"); return m, nil }
                    minv, err1 := strconv.Atoi(parts[0])
                    maxv, err2 := strconv.Atoi(parts[1])
                    if err1 != nil || err2 != nil { m.err = errors.New("invalid numbers"); return m, nil }
                    id := m.selectedID()
                    return m, m.setLimits(id, minv, maxv)
                }
                return m, rescan(m.port, m.baud)
            case tea.KeyEsc:
                m.entering = false; m.status = ""; return m, nil
            }
            // forward to textinput
            var cmd tea.Cmd
            m.input, cmd = m.input.Update(msg)
            return m, cmd
        }

        switch msg.String() {
        case "q", "ctrl+c":
            if m.bus != nil { m.bus.close() }
            return m, tea.Quit
        case "r":
            return m, rescan(m.port, m.baud)
        case "enter":
            id := m.selectedID()
            if id == 0 { return m, nil }
            // toggle torque
            return m, m.toggleTorque(id)
        case "p":
            m.entering = true; m.action = "pos_deg"; m.prompt = "angle (deg): "; m.input.SetValue(""); m.input.Focus()
            return m, nil
        case "u":
            m.entering = true; m.action = "pos_units"; m.prompt = "position (units): "; m.input.SetValue(""); m.input.Focus()
            return m, nil
        case "i":
            m.entering = true; m.action = "set_id"; m.prompt = "new ID: "; m.input.SetValue(""); m.input.Focus()
            return m, nil
        case "l":
            m.entering = true; m.action = "limits"; m.prompt = "min max (units): "; m.input.SetValue(""); m.input.Focus()
            return m, nil
        case "d":
            return m, m.dumpSelected()
        }

    case rescanMsg:
        if msg.err != nil { m.err = msg.err; m.status = msg.err.Error(); return m, nil }
        m.servos = msg.servos
        m.rows = make([]table.Row, 0, len(m.servos))
        for _, s := range m.servos {
            volt := fmt.Sprintf("%.1f", float64(s.Volt10)/10.0)
            tq := "off"; if s.Torque { tq = "on" }
            m.rows = append(m.rows, table.Row{
                fmt.Sprintf("%d", s.ID), fmt.Sprintf("%d", s.Model), fmt.Sprintf("%d", s.Pos), volt, fmt.Sprintf("%d", s.TempC), tq, fmt.Sprintf("%d", s.Min), fmt.Sprintf("%d", s.Max),
            })
        }
        m.table.SetRows(m.rows)
        if len(m.rows) > 0 && m.table.Cursor() >= len(m.rows) { m.table.SetCursor(len(m.rows)-1) }
        m.status = fmt.Sprintf("found %d servos", len(m.rows))
        return m, nil

    case writeDoneMsg:
        if msg.err != nil { m.err = msg.err; m.status = msg.err.Error() }
        return m, rescan(m.port, m.baud)

    case dumpMsg:
        if msg.err != nil { m.err = msg.err; m.status = msg.err.Error() } else { m.status = msg.text }
        return m, nil
    }

    // table + input updates
    var cmds []tea.Cmd
    var cmd tea.Cmd
    m.table, cmd = m.table.Update(msg); cmds = append(cmds, cmd)
    if m.entering { m.input, cmd = m.input.Update(msg); cmds = append(cmds, cmd) }
    return m, tea.Batch(cmds...)
}

func (m appModel) View() string {
    title := lipgloss.NewStyle().Bold(true).Render("rx1tui — Feetech bus (URT‑1)")
    help := "r:refresh  enter:torque  p:pos(deg)  u:pos(units)  i:set id  l:limits  d:dump  q:quit"
    var b strings.Builder
    b.WriteString(title + "\n")
    b.WriteString(m.table.View() + "\n")
    if m.entering {
        b.WriteString("\n" + m.prompt + m.input.View() + "\n")
    }
    if m.err != nil {
        b.WriteString(lipgloss.NewStyle().Foreground(lipgloss.Color("9")).Render("error: "+m.err.Error()) + "\n")
    }
    b.WriteString(lipgloss.NewStyle().Foreground(lipgloss.Color("10")).Render(m.status) + "\n")
    b.WriteString(help)
    return b.String()
}

func (m appModel) selectedID() int {
    if len(m.servos) == 0 { return 0 }
    i := m.table.Cursor()
    if i < 0 || i >= len(m.servos) { return 0 }
    return m.servos[i].ID
}

func (m appModel) toggleTorque(id int) tea.Cmd {
    return func() tea.Msg {
        if m.bus == nil { return writeDoneMsg{err: errors.New("bus not open")} }
        tq, err := m.bus.readRegs(byte(id), regTorqueEnable, 1)
        if err != nil { return writeDoneMsg{err: err} }
        nv := byte(1); if tq[0] != 0 { nv = 0 }
        if err := m.bus.writeRegs(byte(id), regTorqueEnable, nv); err != nil { return writeDoneMsg{err: err} }
        return writeDoneMsg{}
    }
}

func (m appModel) writeGoalPos(id int, units int) tea.Cmd {
    return func() tea.Msg {
        if m.bus == nil { return writeDoneMsg{err: errors.New("bus not open")} }
        lo, hi := toLE16(units)
        if err := m.bus.writeRegs(byte(id), regGoalPosition, lo, hi); err != nil {
            return writeDoneMsg{err: err}
        }
        // optional: gentle move time
        // _ = m.bus.writeRegs(byte(id), regGoalTime, toLE16(300))
        return writeDoneMsg{}
    }
}

func (m appModel) changeID(curID, newID int) tea.Cmd {
    return func() tea.Msg {
        if m.bus == nil { return writeDoneMsg{err: errors.New("bus not open")} }
        // unlock, write ID, lock
        if err := m.bus.writeRegs(byte(curID), regLock, 0); err != nil { return writeDoneMsg{err: err} }
        if err := m.bus.writeRegs(byte(curID), regID, byte(newID)); err != nil { return writeDoneMsg{err: err} }
        if err := m.bus.writeRegs(byte(newID), regLock, 1); err != nil { return writeDoneMsg{err: err} }
        return writeDoneMsg{}
    }
}

func (m appModel) setLimits(id, minv, maxv int) tea.Cmd {
    return func() tea.Msg {
        if m.bus == nil { return writeDoneMsg{err: errors.New("bus not open")} }
        if err := m.bus.writeRegs(byte(id), regLock, 0); err != nil { return writeDoneMsg{err: err} }
        lo, hi := toLE16(minv)
        if err := m.bus.writeRegs(byte(id), regAngleMin, lo, hi); err != nil { return writeDoneMsg{err: err} }
        lo, hi = toLE16(maxv)
        if err := m.bus.writeRegs(byte(id), regAngleMax, lo, hi); err != nil { return writeDoneMsg{err: err} }
        _ = m.bus.writeRegs(byte(id), regLock, 1)
        return writeDoneMsg{}
    }
}

func (m appModel) dumpSelected() tea.Cmd {
    return func() tea.Msg {
        id := m.selectedID()
        if id == 0 || m.bus == nil { return dumpMsg{err: errors.New("no selection/bus")} }
        // read a few key regs
        regs := []struct{ a, n byte; name string }{
            {regModel, 2, "Model"}, {regID, 1, "ID"}, {regBaud, 1, "Baud"},
            {regAngleMin, 2, "Min"}, {regAngleMax, 2, "Max"},
            {regTorqueEnable, 1, "Torque"}, {regGoalPosition, 2, "GoalPos"}, {regGoalTime, 2, "GoalTime"},
            {regPresentPosition, 2, "Pos"}, {regPresentSpeed, 2, "Speed"}, {regPresentVoltage, 1, "Volt10"}, {regPresentTemp, 1, "Temp"},
        }
        var out bytes.Buffer
        for _, r := range regs {
            b, err := m.bus.readRegs(byte(id), r.a, r.n)
            if err != nil { return dumpMsg{err: err} }
            if r.n == 1 {
                fmt.Fprintf(&out, "%s(%d)= %d\n", r.name, r.a, b[0])
            } else if r.n == 2 {
                fmt.Fprintf(&out, "%s(%d)= %d [hex %s]\n", r.name, r.a, le16(b[0], b[1]), strings.ToUpper(hex.EncodeToString(b)))
            } else {
                fmt.Fprintf(&out, "%s(%d)= % X\n", r.name, r.a, b)
            }
        }
        return dumpMsg{text: strings.TrimSpace(out.String())}
    }
}

// ------------------------------ Table styles -----------------------------------

func defaultTableStyles() table.Styles {
    s := table.Styles{}
    s.Header = lipgloss.NewStyle().BorderStyle(lipgloss.NormalBorder()).BorderBottom(true).Bold(true)
    s.Selected = lipgloss.NewStyle().Foreground(lipgloss.Color("229")).Bold(true)
    return s
}

// ---------------------------------- main ---------------------------------------

func main() {
    port := flag.String("port", "/dev/ttyUSB0", "serial port for URT‑1")
    baud := flag.Int("baud", 1000000, "baudrate")
    family := flag.String("family", "sts", "servo family: sts|scs (affects degree mapping only)")
    flag.Parse()

    p := tea.NewProgram(newApp(*port, *baud, *family))
    if _, err := p.Run(); err != nil { log.Fatal(err) }
}
