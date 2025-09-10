# Repository Guidelines

## Project Structure & Module Organization
- `main.go`: Bubble Tea TUI for scanning/controlling Feetech serial‑bus servos via URT‑1.
- `go.mod`, `go.sum`: Go module and dependencies (Go 1.25+).
- Suggested growth pattern: `cmd/rx1tui/` (entry), `internal/bus/` (serial + protocol), `internal/ui/` (model/update/view), `pkg/feetech/` (registers/helpers).

## Build, Test, and Development Commands
- Build: `go build -o rx1tui` — produces the binary.
- Run: `go run . -port /dev/ttyUSB0 -baud 1000000 -family sts` (macOS example: `-port /dev/tty.usbserial-XXXX`).
- Verify format: `go fmt ./...` and `gofmt -s -w .`.
- Static checks: `go vet ./...` (optional: add `golangci-lint run`).

## Coding Style & Naming Conventions
- Formatting: Go defaults (`go fmt`); tabs, import grouping, short receivers.
- Naming: packages are lower‑case; exported types/functions use CamelCase with doc comments; constants for register addresses and instructions live near protocol code.
- Files: UI logic in `internal/ui`, protocol/bus in `internal/bus`; keep functions focused and side‑effect aware.

## Testing Guidelines
- Framework: standard `testing` package; place tests in `*_test.go` next to code.
- Run all tests: `go test ./...`; coverage: `go test -cover ./...` (target ≥80% where practical).
- Hardware/integration: gate with build tag `integration` and skip by default; run via `go test -tags=integration ./internal/...` when URT‑1 is attached.
- Prefer table‑driven tests for packet encode/decode and unit conversions.

## Commit & Pull Request Guidelines
- Commits: imperative mood, concise subject ≤72 chars; include rationale in body when needed. Example: `bus: fix checksum for readReply() under short frames`.
- PRs: clear description, reproduction steps, expected/actual behavior, and screenshots/gifs of the TUI. Link issues (`Fixes #123`). Ensure `go fmt`, `go vet`, and tests pass.

## Security & Configuration Tips
- Serial access: Linux add user to `dialout` (e.g., `sudo usermod -aG dialout $USER`); macOS uses `/dev/tty.*` devices.
- Default baud 1,000,000; confirm adapter supports it. Avoid committing hardware IDs or local paths.
