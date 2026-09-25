# -*- coding: utf-8 -*-
"""
Đếm số lượng ống nước, đầu phun sprinkler và ống gió trong mô hình.

Nhóm theo: Level, Hệ thống (System Type), Family : Type, Kích thước.
Với ống nước và ống gió có thêm tổng chiều dài (m).

Kết quả in ra màn hình và xuất file CSV cạnh file .rvt:
  SoLuong_<tên file>_<thời gian>.csv
"""
from __future__ import print_function

import datetime
import os

import clr
clr.AddReference('RevitAPI')
from Autodesk.Revit.DB import BuiltInCategory, BuiltInParameter, FilteredElementCollector
from System.IO import StreamWriter
from System.Text import UTF8Encoding

# ============================ CẤU HÌNH ============================
ONLY_ACTIVE_VIEW = False   # True = chỉ đếm đối tượng hiển thị trong view hiện hành
# ==================================================================

FT_TO_M = 0.3048

# (nhãn, category, tham số hệ thống, có tính chiều dài?)
TARGETS = [
    (u'Ống nước', BuiltInCategory.OST_PipeCurves,
     BuiltInParameter.RBS_PIPING_SYSTEM_TYPE_PARAM, True),
    (u'Đầu phun sprinkler', BuiltInCategory.OST_Sprinklers,
     BuiltInParameter.RBS_PIPING_SYSTEM_TYPE_PARAM, False),
    (u'Ống gió', BuiltInCategory.OST_DuctCurves,
     BuiltInParameter.RBS_DUCT_SYSTEM_TYPE_PARAM, True),
]


def param_text(el, bip):
    p = el.get_Parameter(bip)
    if p is None or not p.HasValue:
        return u''
    return p.AsValueString() or p.AsString() or u''


def level_name(doc, el):
    lid = el.LevelId
    if lid is None or lid.IntegerValue < 0:
        p = el.get_Parameter(BuiltInParameter.RBS_START_LEVEL_PARAM)
        lid = p.AsElementId() if p is not None else None
    lvl = doc.GetElement(lid) if lid is not None and lid.IntegerValue >= 0 else None
    return lvl.Name if lvl else u'(không có level)'


def type_text(doc, el):
    t = doc.GetElement(el.GetTypeId())
    if t is None:
        return el.Name
    return u'%s : %s' % (t.FamilyName, t.Name)


def collect(doc, view, category):
    col = FilteredElementCollector(doc, view.Id) if view else FilteredElementCollector(doc)
    return list(col.OfCategory(category).WhereElementIsNotElementType())


def csv_cell(v):
    if v is None:
        return ''
    if isinstance(v, float):
        return '%.2f' % v
    s = u'%s' % v
    if any(c in s for c in ',"\n'):
        s = '"' + s.replace('"', '""') + '"'
    return s


def main():
    doc = __revit__.ActiveUIDocument.Document  # noqa: F821 (biến có sẵn trong pyRevit/RPS)
    view = doc.ActiveView if ONLY_ACTIVE_VIEW else None
    print(u'Phạm vi: %s\n' % (u'view "%s"' % view.Name if view else u'toàn bộ mô hình'))

    rows = []
    for label, cat, sys_bip, has_length in TARGETS:
        elements = collect(doc, view, cat)
        groups = {}
        for el in elements:
            key = (level_name(doc, el), param_text(el, sys_bip), type_text(doc, el),
                   param_text(el, BuiltInParameter.RBS_CALCULATED_SIZE) if has_length else u'')
            g = groups.setdefault(key, [0, 0.0])
            g[0] += 1
            if has_length:
                p = el.get_Parameter(BuiltInParameter.CURVE_ELEM_LENGTH)
                if p is not None and p.HasValue:
                    g[1] += p.AsDouble() * FT_TO_M

        total_n = sum(g[0] for g in groups.values())
        total_len = sum(g[1] for g in groups.values())
        header = u'=== %s: %d' % (label, total_n)
        if has_length:
            header += u' đoạn, tổng dài %.1f m' % total_len
        print(header + u' ===')

        for key in sorted(groups):
            n, length = groups[key]
            lvl, system, typ, size = key
            line = u'  %-18s | %-28s | %-40s | %-12s | %5d' % (lvl, system, typ, size, n)
            if has_length:
                line += u' | %8.1f m' % length
            print(line)
            rows.append((label, lvl, system, typ, size, n, length if has_length else None))
        print(u'')

    folder = os.path.dirname(doc.PathName) if doc.PathName else os.path.expanduser('~')
    name = os.path.splitext(os.path.basename(doc.PathName or 'model'))[0]
    stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M')
    csv_path = os.path.join(folder, u'SoLuong_%s_%s.csv' % (name, stamp))
    sw = StreamWriter(csv_path, False, UTF8Encoding(True))
    try:
        sw.WriteLine(u'Hạng mục,Level,Hệ thống,Family : Type,Kích thước,Số lượng,Tổng dài (m)')
        for r in rows:
            sw.WriteLine(u','.join(csv_cell(v) for v in r))
    finally:
        sw.Close()
    print(u'Đã xuất báo cáo: %s' % csv_path)


main()
