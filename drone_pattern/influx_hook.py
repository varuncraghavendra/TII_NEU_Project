# influx_hook.py
import os
import time
from collections import defaultdict
from typing import Dict, List, Tuple, Optional

import influxdb_client
from influxdb_client.client.exceptions import InfluxDBError

# -------------------- Config (env-driven) --------------------
INFLUX_URL    = os.environ.get("INFLUXDB_URL",    "http://10.100.14.130:8086")
INFLUX_ORG    = os.environ.get("INFLUXDB_ORG",    "Northeastern")  # <— fix typo here
INFLUX_BUCKET = os.environ.get("INFLUXDB_BUCKET", "digiran-demo")
INFLUX_TOKEN  = os.environ.get("INFLUXDB_TOKEN")  # must be set if auth enabled

# Query window (seconds)
QUERY_WINDOW_S = int(os.environ.get("INFLUXDB_QUERY_WINDOW_S", "5"))  # e.g. 5s window

# Networking robustness
HTTP_TIMEOUT_MS   = int(os.environ.get("INFLUXDB_HTTP_TIMEOUT_MS", "30000"))  # 30s
RETRIES           = int(os.environ.get("INFLUXDB_RETRIES", "2"))             # total tries = RETRIES+1
RETRY_BACKOFF_SEC = float(os.environ.get("INFLUXDB_RETRY_BACKOFF_SEC", "2.0"))

# -------------------- Helpers --------------------
def _connect() -> influxdb_client.InfluxDBClient:
    """
    Create a client with an explicit timeout. Raises on failure.
    """
    if not INFLUX_URL:
        raise RuntimeError("INFLUXDB_URL is not set")
    if not INFLUX_ORG:
        raise RuntimeError("INFLUXDB_ORG is not set")
    # Token may be optional if your InfluxDB is unsecured / local
    return influxdb_client.InfluxDBClient(
        url=INFLUX_URL,
        token=INFLUX_TOKEN,
        org=INFLUX_ORG,
        timeout=HTTP_TIMEOUT_MS
    )

def _health_ok(client: influxdb_client.InfluxDBClient) -> bool:
    try:
        h = client.health()
        return getattr(h, "status", "fail").lower() == "pass"
    except Exception:
        return False

def _build_flux_query(bucket: str, window_s: int) -> str:
    # Single-shot query over the last `window_s` seconds; sorted by _time.
    # Keep all RSRP points tagged with RNTI.
    return f"""
from(bucket: "{bucket}")
  |> range(start: -{window_s}s)
  |> filter(fn: (r) => r._measurement == "oai_metrics")
  |> filter(fn: (r) => r._field == "RSRP")
  |> filter(fn: (r) => exists r.RNTI)
  |> sort(columns: ["_time"])
"""

def _query_once(query_api, flux: str) -> Dict[str, List[float]]:
    """
    Execute one Flux query; return raw per-RNTI lists of RSRP values.
    """
    rsrp_data: Dict[str, List[float]] = defaultdict(list)
    tables = query_api.query(query=flux, org=INFLUX_ORG)
    for table in tables:
        for record in table.records:
            rnti = record.values.get("RNTI")
            rsrp_value = record.get_value()
            if rnti is None:
                continue
            if rsrp_value is None:
                continue
            rsrp_data[str(rnti)].append(float(rsrp_value))
    return rsrp_data

def _aggregate(rsrp_data: Dict[str, List[float]]) -> Dict[str, float]:
    """
    Average per RNTI; if any value is 0 => -inf for that RNTI.
    """
    out: Dict[str, float] = {}
    for rnti, vals in rsrp_data.items():
        if not vals:
            continue
        if any(v == 0.0 for v in vals):
            out[rnti] = float("-inf")
        else:
            out[rnti] = sum(vals) / len(vals)
    return out

# -------------------- Public API --------------------
def get_average_rsrp_3s() -> Dict[str, float]:
    """
    Collects RSRP over the last QUERY_WINDOW_S seconds (default 5s) in a single query.
    Returns { RNTI: avg_rsrp } with the special rule that if any value is 0 for an RNTI,
    that average becomes -inf.

    On failure, returns {}.
    """
    flux = _build_flux_query(INFLUX_BUCKET, QUERY_WINDOW_S)

    attempt = 0
    last_err: Optional[Exception] = None

    while attempt <= RETRIES:
        try:
            client = _connect()

            if not _health_ok(client):
                raise ConnectionError(f"InfluxDB health check failed at {INFLUX_URL}")

            query_api = client.query_api()
            rsrp_data = _query_once(query_api, flux)
            client.close()

            # Empty is valid (no points), propagate to caller to decide fallback
            return _aggregate(rsrp_data)

        except (InfluxDBError, ConnectionError, Exception) as e:
            last_err = e
            # Mirror your original logging style so you can see exact reason.
            print(f"Error collecting data: {e}")

            # Close client if it was created
            try:
                client.close()  # type: ignore
            except Exception:
                pass

            if attempt < RETRIES:
                time.sleep(RETRY_BACKOFF_SEC * (attempt + 1))  # simple linear backoff
                attempt += 1
                continue
            break

    # All attempts failed
    return {}
