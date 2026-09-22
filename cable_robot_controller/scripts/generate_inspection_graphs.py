#!/usr/bin/env python3
"""Generate monitoring graphs from inspection_log.csv."""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from xml.sax.saxutils import escape
from zipfile import ZIP_DEFLATED, ZipFile

import matplotlib.pyplot as plt
import numpy as np


FIELDNAMES = [
    "time",
    "robot_x",
    "robot_y",
    "fault_type",
    "cable_distance",
    "repair_status",
]


@dataclass
class LogRow:
    timestamp: datetime
    robot_x: float
    robot_y: float
    fault_type: str
    cable_distance: float
    repair_status: str


def parse_time(raw: str) -> datetime:
    raw = raw.strip()
    if not raw:
        return datetime.fromtimestamp(0)
    try:
        return datetime.fromisoformat(raw)
    except ValueError:
        try:
            return datetime.fromtimestamp(float(raw))
        except ValueError:
            return datetime.fromtimestamp(0)


def row_from_record(row: dict[str, str]) -> LogRow:
    return LogRow(
        timestamp=parse_time(row.get("time", "")),
        robot_x=float(row.get("robot_x", 0.0) or 0.0),
        robot_y=float(row.get("robot_y", 0.0) or 0.0),
        fault_type=(row.get("fault_type", "") or "").strip(),
        cable_distance=float(row.get("cable_distance", 0.0) or 0.0),
        repair_status=(row.get("repair_status", "") or "").strip(),
    )


def rows_from_records(records: list[dict[str, str]]) -> list[LogRow]:
    return [row_from_record(record) for record in records]


def read_rows(log_path: Path) -> list[LogRow]:
    rows: list[LogRow] = []
    with log_path.open("r", newline="", encoding="utf-8") as file_obj:
        reader = csv.DictReader(file_obj)
        for row in reader:
            rows.append(row_from_record(row))
    return rows


def _excel_column_name(index: int) -> str:
    result = ""
    current = index
    while current > 0:
        current, remainder = divmod(current - 1, 26)
        result = chr(65 + remainder) + result
    return result


def _xlsx_inline_cell(cell_ref: str, value: str) -> str:
    return (
        f'<c r="{cell_ref}" t="inlineStr"><is><t>{escape(value)}</t></is></c>'
    )


def _xlsx_number_cell(cell_ref: str, value: float) -> str:
    return f'<c r="{cell_ref}" t="n"><v>{value:.6f}</v></c>'


def write_excel_report(rows: list[LogRow], output_path: Path) -> Path:
    output_path.parent.mkdir(parents=True, exist_ok=True)

    headers = list(FIELDNAMES)
    sheet_rows = [
        [
            row.timestamp.isoformat(timespec="seconds"),
            f"{row.robot_x:.3f}",
            f"{row.robot_y:.3f}",
            row.fault_type,
            f"{row.cable_distance:.3f}",
            row.repair_status,
        ]
        for row in rows
    ]

    row_xml: list[str] = []
    header_cells = [
        _xlsx_inline_cell(f"{_excel_column_name(idx + 1)}1", header)
        for idx, header in enumerate(headers)
    ]
    row_xml.append(f'<row r="1">{"".join(header_cells)}</row>')

    for row_idx, values in enumerate(sheet_rows, start=2):
        cells: list[str] = []
        for col_idx, value in enumerate(values, start=1):
            cell_ref = f"{_excel_column_name(col_idx)}{row_idx}"
            if col_idx in {2, 3, 5}:
                cells.append(_xlsx_number_cell(cell_ref, float(value)))
            else:
                cells.append(_xlsx_inline_cell(cell_ref, value))
        row_xml.append(f'<row r="{row_idx}">{"".join(cells)}</row>')

    last_column = _excel_column_name(len(headers))
    last_row = max(1, len(sheet_rows) + 1)
    sheet_xml = (
        '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
        '<worksheet xmlns="http://schemas.openxmlformats.org/spreadsheetml/2006/main">'
        f"<dimension ref=\"A1:{last_column}{last_row}\"/>"
        "<sheetViews><sheetView workbookViewId=\"0\"/></sheetViews>"
        "<sheetFormatPr defaultRowHeight=\"15\"/>"
        f"<sheetData>{''.join(row_xml)}</sheetData>"
        "</worksheet>"
    )

    workbook_xml = (
        '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
        '<workbook xmlns="http://schemas.openxmlformats.org/spreadsheetml/2006/main" '
        'xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships">'
        '<sheets><sheet name="inspection_points" sheetId="1" r:id="rId1"/></sheets>'
        "</workbook>"
    )
    workbook_rels_xml = (
        '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
        '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">'
        '<Relationship Id="rId1" '
        'Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/worksheet" '
        'Target="worksheets/sheet1.xml"/>'
        "</Relationships>"
    )
    root_rels_xml = (
        '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
        '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">'
        '<Relationship Id="rId1" '
        'Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/officeDocument" '
        'Target="xl/workbook.xml"/>'
        "</Relationships>"
    )
    content_types_xml = (
        '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
        '<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">'
        '<Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>'
        '<Default Extension="xml" ContentType="application/xml"/>'
        '<Override PartName="/xl/workbook.xml" '
        'ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.sheet.main+xml"/>'
        '<Override PartName="/xl/worksheets/sheet1.xml" '
        'ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.worksheet+xml"/>'
        "</Types>"
    )

    with ZipFile(output_path, "w", compression=ZIP_DEFLATED) as zip_file:
        zip_file.writestr("[Content_Types].xml", content_types_xml)
        zip_file.writestr("_rels/.rels", root_rels_xml)
        zip_file.writestr("xl/workbook.xml", workbook_xml)
        zip_file.writestr("xl/_rels/workbook.xml.rels", workbook_rels_xml)
        zip_file.writestr("xl/worksheets/sheet1.xml", sheet_xml)

    return output_path


def create_cable_health_graph(rows: list[LogRow], output_dir: Path) -> Path:
    max_distance = max((row.cable_distance for row in rows), default=1.0)
    max_distance = max(1.0, max_distance)

    x = np.linspace(0.0, max_distance, 400)
    health = np.ones_like(x)

    for row in rows:
        if row.fault_type == "CORROSION":
            target = 0.5
        elif row.fault_type == "CABLE BREAK":
            target = 0.0
        else:
            continue

        idx = int(np.argmin(np.abs(x - row.cable_distance)))
        left = max(0, idx - 2)
        right = min(len(x), idx + 3)
        health[left:right] = np.minimum(health[left:right], target)

    fig, ax = plt.subplots(figsize=(9, 4.8))
    ax.plot(x, health, color="#1f77b4", linewidth=2.2)
    ax.set_title("Cable Health vs Distance")
    ax.set_xlabel("Cable Distance (m)")
    ax.set_ylabel("Cable Health Status")
    ax.set_ylim(-0.05, 1.05)
    ax.set_yticks([0.0, 0.5, 1.0])
    ax.set_yticklabels(["Break (0)", "Corrosion (0.5)", "Healthy (1)"])
    ax.grid(alpha=0.35)
    fig.tight_layout()

    output_path = output_dir / "graph_cable_health_vs_distance.png"
    fig.savefig(output_path, dpi=150)
    plt.close(fig)
    return output_path


def create_fault_timeline_graph(rows: list[LogRow], output_dir: Path) -> Path:
    corrosion_times = [row.timestamp for row in rows if row.fault_type == "CORROSION"]
    break_times = [row.timestamp for row in rows if row.fault_type == "CABLE BREAK"]

    fig, ax = plt.subplots(figsize=(9, 4.8))
    if corrosion_times:
        ax.scatter(
            corrosion_times,
            [1] * len(corrosion_times),
            label="corrosion event",
            marker="o",
            s=64,
            color="#d98c00",
        )
    if break_times:
        ax.scatter(
            break_times,
            [2] * len(break_times),
            label="break event",
            marker="x",
            s=72,
            color="#c62828",
            linewidths=2.0,
        )

    if not corrosion_times and not break_times:
        ax.text(
            0.5,
            0.5,
            "No fault events logged",
            transform=ax.transAxes,
            ha="center",
            va="center",
        )

    ax.set_title("Fault Timeline")
    ax.set_xlabel("Time")
    ax.set_ylabel("Fault Events")
    ax.set_yticks([1, 2])
    ax.set_yticklabels(["corrosion event", "break event"])
    ax.grid(alpha=0.35)
    if corrosion_times or break_times:
        ax.legend(loc="best")
    fig.autofmt_xdate(rotation=20)
    fig.tight_layout()

    output_path = output_dir / "graph_fault_timeline.png"
    fig.savefig(output_path, dpi=150)
    plt.close(fig)
    return output_path


def create_robot_path_graph(rows: list[LogRow], output_dir: Path) -> Path:
    x = [row.robot_x for row in rows]
    y = [row.robot_y for row in rows]

    fig, ax = plt.subplots(figsize=(7.2, 6.0))
    if x and y:
        ax.plot(x, y, color="#00695c", linewidth=2.0, label="inspection path")
        ax.scatter([x[0]], [y[0]], color="#2e7d32", s=65, label="start")
        ax.scatter([x[-1]], [y[-1]], color="#c62828", s=65, label="latest")
        ax.legend(loc="best")
    else:
        ax.text(
            0.5,
            0.5,
            "No robot path samples logged",
            transform=ax.transAxes,
            ha="center",
            va="center",
        )

    ax.set_title("Robot Inspection Path")
    ax.set_xlabel("Robot X Position")
    ax.set_ylabel("Robot Y Position")
    ax.grid(alpha=0.35)
    ax.set_aspect("equal", adjustable="datalim")
    fig.tight_layout()

    output_path = output_dir / "graph_robot_inspection_path.png"
    fig.savefig(output_path, dpi=150)
    plt.close(fig)
    return output_path


def generate_graphs(rows: list[LogRow], output_dir: Path) -> tuple[Path, Path, Path]:
    output_dir.mkdir(parents=True, exist_ok=True)
    out1 = create_cable_health_graph(rows, output_dir)
    out2 = create_fault_timeline_graph(rows, output_dir)
    out3 = create_robot_path_graph(rows, output_dir)
    return out1, out2, out3


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate cable inspection monitoring graphs.")
    parser.add_argument(
        "--log-file",
        default="/home/kousik/inspection_log.csv",
        help="Path to inspection_log.csv",
    )
    parser.add_argument(
        "--excel-file",
        default="/home/kousik/inspection_points.xlsx",
        help="Path to write Excel workbook with sampled points",
    )
    parser.add_argument(
        "--output-dir",
        default="/home/kousik",
        help="Directory to write graph PNG files",
    )
    args = parser.parse_args()

    log_path = Path(args.log_file)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    if not log_path.exists():
        raise FileNotFoundError(f"Log file not found: {log_path}")

    rows = read_rows(log_path)
    if not rows:
        raise RuntimeError(f"No rows found in log file: {log_path}")

    excel_path = write_excel_report(rows, Path(args.excel_file))
    out1, out2, out3 = generate_graphs(rows, output_dir)

    print(f"Saved: {excel_path}")
    print(f"Saved: {out1}")
    print(f"Saved: {out2}")
    print(f"Saved: {out3}")


if __name__ == "__main__":
    main()
