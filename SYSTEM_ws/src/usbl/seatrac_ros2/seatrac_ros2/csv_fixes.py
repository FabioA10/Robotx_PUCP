"""CSV input for normalized USBL relative-position recordings."""

import csv
from pathlib import Path
from typing import Dict, Iterable, List, Optional

from seatrac_ros2.fix_validation import UsblFix


class CsvFormatError(ValueError):
    """Describe a malformed recording with its input line."""


def load_csv_fixes(path: str) -> List[UsblFix]:
    """Load and validate the syntax of a normalized USBL CSV file."""
    csv_path = Path(path).expanduser()
    with csv_path.open("r", encoding="utf-8", newline="") as stream:
        rows = list(_non_comment_lines(stream))
    if not rows:
        raise CsvFormatError("CSV is empty")

    reader = csv.DictReader(rows)
    if reader.fieldnames is None:
        raise CsvFormatError("CSV header is missing")
    fieldnames = {name.strip() for name in reader.fieldnames if name}
    timestamp_name = "timestamp_s" if "timestamp_s" in fieldnames else "stamp_s"
    required = {
        timestamp_name,
        "body_forward_m",
        "body_left_m",
    }
    missing = sorted(required - fieldnames)
    if missing:
        raise CsvFormatError("missing columns: " + ", ".join(missing))

    samples = []
    for line_number, row in enumerate(reader, start=2):
        normalized = {
            str(key).strip(): str(value).strip()
            for key, value in row.items()
            if key is not None and value is not None
        }
        try:
            samples.append(
                UsblFix(
                    timestamp_s=_float(normalized, timestamp_name),
                    x_m=_float(normalized, "body_forward_m"),
                    y_m=_float(normalized, "body_left_m"),
                    z_m=_float(
                        normalized, "body_up_m", default=0.0
                    ),
                    beacon_id=_integer_optional(
                        normalized, "beacon_id", "src_id"
                    ),
                    range_m=_float_optional(normalized, "range_m"),
                    fit_error=_float_optional(normalized, "fit_error"),
                    rssi_db=_float_optional(normalized, "rssi_db"),
                    position_valid=_boolean(
                        normalized, "position_valid", "valid", default=True
                    ),
                    position_filter_error=_boolean(
                        normalized,
                        "position_filter_error",
                        default=False,
                    ),
                )
            )
        except (TypeError, ValueError) as error:
            raise CsvFormatError(
                f"line {line_number}: {error}"
            ) from error
    if not samples:
        raise CsvFormatError("CSV contains no data rows")
    return samples


def _non_comment_lines(lines: Iterable[str]) -> Iterable[str]:
    for line in lines:
        if line.strip() and not line.lstrip().startswith("#"):
            yield line


def _first(row: Dict[str, str], *names: str) -> Optional[str]:
    for name in names:
        value = row.get(name, "")
        if value != "":
            return value
    return None


def _float(
    row: Dict[str, str],
    name: str,
    default: Optional[float] = None,
) -> float:
    value = _first(row, name)
    if value is None:
        if default is not None:
            return default
        raise ValueError(f"empty required field {name}")
    return float(value)


def _float_optional(row: Dict[str, str], *names: str) -> Optional[float]:
    value = _first(row, *names)
    return None if value is None else float(value)


def _integer_optional(row: Dict[str, str], *names: str) -> Optional[int]:
    value = _first(row, *names)
    return None if value is None else int(value)


def _boolean(
    row: Dict[str, str],
    *names: str,
    default: bool,
) -> bool:
    value = _first(row, *names)
    if value is None:
        return default
    normalized = value.lower()
    if normalized in {"1", "true", "yes", "y"}:
        return True
    if normalized in {"0", "false", "no", "n"}:
        return False
    raise ValueError(f"invalid boolean for {names[0]}: {value!r}")
