# Raspberry Pi DAQ: CAN encoder/inclinometer + ADS1263 → SQLite

Raspberry Pi 4 + MCP2515 CAN HAT + Waveshare High-Precision AD HAT (ADS1263).
The program samples at a fixed period (100 ms by default). Each sample merges
the following into one row with a timestamp and writes it to SQLite:

- ADC voltage
- encoder distance from CAN (ID `0x2A0`)
- inclinometer X/Y from CAN (ID `0x28A`/`0x28C`)

```
.
├── src/daq_main.cpp            # main program (CAN + ADC + DB)
├── src/db_writer.cpp           # asynchronous SQLite writer
├── include/                    # shared headers (CAN socket, sensor decoding, signals)
├── drivers/                    # ADS1263 driver (based on Waveshare, modified)
├── tools/can_module_test.cpp   # CAN-only test (formerly CAN_Module.cpp)
├── tools/can_2ch_load_test.cpp # 2-channel CAN + CPU load test (formerly 2CH_OC.cpp)
├── tests/                      # unit tests that run on a PC
└── deploy/daq.service          # systemd service
```

## Build

```bash
sudo apt install build-essential libsqlite3-dev sqlite3
# bcm2835 library: http://www.airspayce.com/mikem/bcm2835/ (install per its instructions)
make            # produces build/daq, build/can_module_test, build/can_2ch_load_test
make test       # unit tests that need no hardware
```

## Run

```bash
sudo ip link set can0 type can bitrate 125000
sudo ip link set up can0

sudo ./build/daq -d daq.db -i can0 -p 100 --vref 5.0
sudo ./build/daq --no-adc -i can0          # CAN + DB only, without the ADC
```

Stop with Ctrl+C (or `systemctl stop`). Rows still in the queue are written
before the program exits.

| Option | Default | Description |
|---|---|---|
| `-d, --db` | `daq.db` | SQLite file |
| `-i, --iface` | `can0` | CAN interface |
| `-p, --period` | `100` | sampling period (ms) |
| `-c, --channel` | `0` | ADC1 input channel (AIN0–AIN10, single-ended vs. AINCOM) |
| `-r, --vref` | `5.0` | ADC reference voltage. **Measure AVDD–AVSS with a multimeter and pass the measured value** |
| `--sync-id`, `--sync-ms` | `0x80`, `100` | SYNC frame ID and period (`--sync-ms 0` disables it) |
| `--stale-ms` | `500` | CAN data older than this gets a STALE flag |
| `-n, --note` | | note saved in the `runs` table |
| `-q, --quiet` | | do not print each sample to the console |

To run it as a boot-time service, see `deploy/daq.service`.

## Database

```
runs     (id, started_ts_ms, ended_ts_ms, hostname, can_iface, period_ms, adc_channel, adc_vref, note)
samples  (id, run_id, ts_ms, mono_ms, adc_raw, adc_v, enc_counts, enc_dist,
          inc_x_deg, inc_y_deg, enc_age_ms, inc_age_ms, flags)
samples_local  -- view: samples with ts_ms converted to local time
```

- `ts_ms`: UTC epoch milliseconds. This is the shared time key for joining
  other data later.
- `mono_ms`: time since program start. It is unaffected by system clock
  corrections, so use it to analyze sample intervals.
- `enc_age_ms` / `inc_age_ms`: how long before this sample the most recent
  CAN frame arrived. CAN data and ADC data are sampled at different times, so
  this column shows how far apart they are.
- `adc_raw`: the raw ADC code. If the reference voltage changes later,
  recompute with `adc_raw / 2^31 * vref`.
- A value that is missing or failed to read is stored as `NULL`, not 0.

`flags` bits:

| bit | value | meaning |
|---|---|---|
| 0 | 1 | ADC read error (timeout/CRC) |
| 1 | 2 | encoder data stale or not received yet |
| 2 | 4 | inclinometer data stale or not received yet |
| 3 | 8 | system clock not NTP-synchronized (`ts_ms` is unreliable) |
| 4 | 16 | previous period overran the target period |
| 5 | 32 | run with `--no-adc` |

Example queries:

```sql
-- latest 10 rows
SELECT * FROM samples_local ORDER BY id DESC LIMIT 10;
-- valid rows only, in a time range
SELECT local_time, adc_v, enc_dist, inc_x_deg, inc_y_deg
FROM samples_local
WHERE flags & 7 = 0
  AND ts_ms BETWEEN strftime('%s','2026-11-23 09:00:00','utc')*1000
                AND strftime('%s','2026-11-23 10:00:00','utc')*1000;
-- error counts per run
SELECT run_id, COUNT(*) n, SUM(flags & 1 > 0) adc_err FROM samples GROUP BY run_id;
```

```python
import sqlite3, pandas as pd
df = pd.read_sql("SELECT * FROM samples WHERE run_id = (SELECT MAX(id) FROM runs)",
                 sqlite3.connect("daq.db"))
df["time"] = pd.to_datetime(df.ts_ms, unit="ms", utc=True).dt.tz_convert("Asia/Seoul")
```

The database uses WAL mode, so other processes can read it while it is being
written. At 100 ms there are about 860,000 rows per day (on the order of tens
of MB).

### Time accuracy

The Raspberry Pi has no RTC. If it boots without a network connection, the
system clock can be wrong. Such rows get flag 8. If you need accurate absolute
time, add an RTC module or use NTP/`chrony`.

## SPI wiring notes (ADC + CAN sharing SPI0)

The ADS1263 HAT uses the SPI0 data lines with GPIO22 as CS. If the CAN HAT is
on SPI0 as well, the two devices share a bus.

- **Default (`ADC_KERNEL_CS=0`)**: user space toggles GPIO22. Each ADC command
  is sent as a single SPI message, so the kernel cannot interleave a CAN
  transfer in the middle of it. A small window remains between the GPIO toggle
  and the transfer. If a read gets corrupted, the ADS1263 checksum catches it
  and the row is stored as `ADC_ERROR` (no bad value is saved).
- **Recommended (`ADC_KERNEL_CS=1`)**: the kernel drives GPIO22 as the CS pin,
  which removes the window entirely. Configure the device tree so the ADC gets
  its own spidev with CS=GPIO22, then build with `make ADC_KERNEL_CS=1
  ADC_SPI_DEVICE=/dev/spidevX.Y`. Example: if the CAN HAT only uses CE0, you
  can use `dtoverlay=spi0-2cs,cs1_pin=22`. **First check which SPI/CS lines
  the CAN HAT overlay actually uses** (`/boot/firmware/config.txt`,
  `ls /dev/spidev*`). Some 2-channel CAN HATs also use GPIO22 as an interrupt
  pin, so check the HAT manual for pin conflicts too.
