# Contributing to Bako OBD ISS

## Branch naming

| Type | Pattern | Example |
|------|---------|---------|
| Feature | `feature/<short-desc>` | `feature/cloud-ingest` |
| Bug fix | `fix/<short-desc>` | `fix/can-endianness` |
| Report/docs | `docs/<short-desc>` | `docs/report-section-3` |
| Firmware | `firmware/<short-desc>` | `firmware/sim800l-gprs` |

All branches must be cut from `main`.

## Commit style

```
<type>: <short summary>

[optional body]
```

Types: `feat`, `fix`, `docs`, `firmware`, `refactor`, `test`, `chore`

Example: `feat: add cloud ingest endpoint with SQLite storage`

## Domain ownership

| Domain | Owner |
|--------|-------|
| `backend/` | backend team |
| `firmware/` | embedded team |
| `frontend/` | frontend team |
| `data/` | data / analysis |
| `report/` | academic report |

Do not edit another domain's files without coordination.

## Pull request checklist

- [ ] Branch is up to date with `main`
- [ ] Code runs without errors
- [ ] No secrets or API keys committed
- [ ] PR description explains *why*, not just *what*

## Setting up locally

```bash
cd backend
python -m venv venv
source venv/bin/activate
pip install -r requirements.txt
python server.py
```

Open `http://localhost:8765` in your browser.

### Cloud mode

```bash
# Set an API key (optional, defaults to bako-bms-2024)
export BMS_API_KEY=your-secret-key
python server.py
```

The ESP32 firmware POSTs to `POST /api/ingest` with header `X-Api-Key`.

## Environment variables

| Variable | Default | Purpose |
|----------|---------|---------|
| `BMS_API_KEY` | `bako-bms-2024` | API key for ESP32 → cloud ingest |
