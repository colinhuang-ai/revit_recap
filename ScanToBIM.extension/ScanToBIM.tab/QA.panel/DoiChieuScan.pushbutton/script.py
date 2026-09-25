# -*- coding: utf-8 -*-
"""
Đối chiếu mô hình Revit với point cloud (kiểm tra chất lượng Scan to BIM).

Cách dùng:
  1. Chèn point cloud vào mô hình: Insert > Point Cloud > chọn file .rcs/.rcp,
     Positioning = "Auto - Origin to Origin" (hoặc "Shared Coordinates" nếu có).
  2. Mở một view 3D (nên dùng Section Box để giới hạn vùng kiểm tra).
  3. Chọn các đối tượng cần kiểm tra (không chọn gì = kiểm tra mọi đối tượng
     hỗ trợ đang hiển thị trong view hiện hành), rồi chạy script.

Kết quả:
  - File CSV cạnh file .rvt: DoiChieu_<tên file>_<thời gian>.csv
  - Tô màu trong view hiện hành (nếu COLORIZE = True):
      xanh lá = OK / CO_DIEM, cam = LECH, tím = SAI_KICH_THUOC,
      vàng = THIEU_MOT_PHAN, đỏ = KHONG_CO_DIEM

Trạng thái:
  OK              bề mặt mô hình khớp với điểm quét trong dung sai
  LECH            mô hình lệch vị trí so với điểm quét quá dung sai
  SAI_KICH_THUOC  (ống tròn) đường kính đo từ điểm quét khác mô hình quá dung sai
  THIEU_MOT_PHAN  chỉ một phần chiều dài có điểm quét (bị che khuất hoặc vẽ thừa)
  KHONG_CO_DIEM   không có điểm quét quanh đối tượng (vẽ thừa, bị che, hoặc
                  point cloud chưa căn đúng vị trí)
  CO_DIEM         (thiết bị, phụ kiện, kết cấu) có điểm quét trong bounding box
"""
from __future__ import print_function

import datetime
import math
import os
import time

import clr
clr.AddReference('RevitAPI')
from Autodesk.Revit.DB import (
    BuiltInCategory, BuiltInParameter, Color, ElementId, FillPatternElement,
    FilteredElementCollector, Line, LocationCurve, OverrideGraphicSettings,
    Plane, PointCloudInstance, Transaction, XYZ)
from Autodesk.Revit.DB.PointClouds import PointCloudFilterFactory
from System.Collections.Generic import List
from System.IO import StreamWriter
from System.Text import UTF8Encoding

# ============================ CẤU HÌNH ============================
TOL_MM = 30.0          # dung sai cho phép giữa bề mặt mô hình và điểm quét
BAND_MM = 150.0        # chỉ xét điểm cách bề mặt mô hình không quá khoảng này
AVG_DIST_MM = 15.0     # mật độ lấy mẫu điểm (nhỏ hơn = chính xác hơn nhưng chậm hơn)
MAX_POINTS = 5000      # số điểm tối đa cho mỗi lần truy vấn
CHUNK_MM = 2000.0      # chia đối tượng dài thành từng đoạn để truy vấn
MIN_POINTS = 30        # ít hơn số này coi như không có dữ liệu quét
BIN_MM = 300.0         # độ dài mỗi ô khi tính độ phủ theo chiều dài
MIN_BIN_POINTS = 5     # số điểm tối thiểu để một ô được coi là có dữ liệu
COVERAGE_OK = 0.6      # độ phủ tối thiểu để không bị đánh dấu THIEU_MOT_PHAN
COLORIZE = True        # tô màu kết quả trong view hiện hành
# ==================================================================

FT_TO_MM = 304.8

LINEAR_CATS = {
    int(BuiltInCategory.OST_PipeCurves): u'Ống nước',
    int(BuiltInCategory.OST_DuctCurves): u'Ống gió',
    int(BuiltInCategory.OST_Conduit): u'Ống luồn dây',
    int(BuiltInCategory.OST_CableTray): u'Máng cáp',
}
BOX_CATS = {
    int(BuiltInCategory.OST_PipeFitting): u'Phụ kiện ống',
    int(BuiltInCategory.OST_PipeAccessory): u'Van/phụ kiện đường ống',
    int(BuiltInCategory.OST_DuctFitting): u'Phụ kiện ống gió',
    int(BuiltInCategory.OST_Sprinklers): u'Đầu phun sprinkler',
    int(BuiltInCategory.OST_MechanicalEquipment): u'Thiết bị cơ',
    int(BuiltInCategory.OST_ElectricalEquipment): u'Thiết bị điện',
    int(BuiltInCategory.OST_PlumbingFixtures): u'Thiết bị vệ sinh',
    int(BuiltInCategory.OST_StructuralFraming): u'Dầm',
    int(BuiltInCategory.OST_StructuralColumns): u'Cột',
}

STATUS_COLORS = {
    'OK': Color(0, 170, 0),
    'CO_DIEM': Color(0, 170, 0),
    'LECH': Color(255, 130, 0),
    'SAI_KICH_THUOC': Color(160, 60, 200),
    'THIEU_MOT_PHAN': Color(230, 200, 0),
    'KHONG_CO_DIEM': Color(220, 0, 0),
}


def to_ft(v_mm):
    return v_mm / FT_TO_MM


def to_mm(v_ft):
    return v_ft * FT_TO_MM


def median(values):
    if not values:
        return None
    s = sorted(values)
    n = len(s)
    if n % 2:
        return s[n // 2]
    return 0.5 * (s[n // 2 - 1] + s[n // 2])


def get_double(el, bip):
    p = el.get_Parameter(bip)
    if p is not None and p.HasValue:
        v = p.AsDouble()
        if v > 0:
            return v
    return None


# ------------------------- Point cloud ----------------------------

def inside_box(p, lo, hi, eps=1e-6):
    return (lo[0] - eps <= p[0] <= hi[0] + eps and
            lo[1] - eps <= p[1] <= hi[1] + eps and
            lo[2] - eps <= p[2] <= hi[2] + eps)


class CloudSource(object):
    """Bọc một PointCloudInstance; trả về điểm trong hệ toạ độ cục bộ của một hộp."""

    # (filter theo toạ độ cục bộ của point cloud?, có áp Transform cho điểm trả về?)
    MODES = ((False, True), (False, False), (True, True), (True, False))

    def __init__(self, inst):
        self.inst = inst
        self.T = inst.GetTotalTransform()
        self.T_inv = self.T.Inverse
        # Tài liệu API không nói rõ hệ toạ độ của filter và của điểm trả về, nên
        # nếu point cloud có Transform thì tự dò quy ước ở lần truy vấn đầu tiên.
        self.mode = (False, True) if self.T.IsIdentity else None

    def _make_filter(self, origin, axes, lo, hi, local):
        planes = List[Plane]()
        for i in range(3):
            ax = axes[i]
            p_lo = origin.Add(ax.Multiply(lo[i]))
            p_hi = origin.Add(ax.Multiply(hi[i]))
            n_lo, n_hi = ax, ax.Negate()
            if local:
                p_lo, p_hi = self.T_inv.OfPoint(p_lo), self.T_inv.OfPoint(p_hi)
                n_lo, n_hi = self.T_inv.OfVector(n_lo), self.T_inv.OfVector(n_hi)
            # Filter giữ lại các điểm nằm phía dương của mọi mặt phẳng
            planes.Add(Plane.CreateByNormalAndOrigin(n_lo, p_lo))
            planes.Add(Plane.CreateByNormalAndOrigin(n_hi, p_hi))
        return PointCloudFilterFactory.CreateMultiPlaneFilter(planes)

    def _query(self, origin, axes, lo, hi, max_points, local, apply_t):
        flt = self._make_filter(origin, axes, lo, hi, local)
        pts = self.inst.GetPoints(flt, to_ft(AVG_DIST_MM), max_points)
        d, w, h = axes
        ox, oy, oz = origin.X, origin.Y, origin.Z
        T = self.T
        out = []
        for p in pts:
            x, y, z = float(p.X), float(p.Y), float(p.Z)
            if apply_t:
                q = T.OfPoint(XYZ(x, y, z))
                x, y, z = q.X, q.Y, q.Z
            rx, ry, rz = x - ox, y - oy, z - oz
            out.append((rx * d.X + ry * d.Y + rz * d.Z,
                        rx * w.X + ry * w.Y + rz * w.Z,
                        rx * h.X + ry * h.Y + rz * h.Z))
        return out

    def points_in_box(self, origin, axes, lo, hi, max_points=MAX_POINTS):
        """Trả về list (t, u, v): toạ độ điểm theo 3 trục axes, gốc origin."""
        if self.mode is not None:
            pts = self._query(origin, axes, lo, hi, max_points, *self.mode)
            return [p for p in pts if inside_box(p, lo, hi)]

        best, best_score, best_mode = [], -1.0, None
        for mode in self.MODES:
            try:
                pts = self._query(origin, axes, lo, hi, max_points, *mode)
            except Exception:
                continue
            if not pts:
                continue
            score = sum(1 for p in pts if inside_box(p, lo, hi)) / float(len(pts))
            if score > best_score:
                best, best_score, best_mode = pts, score, mode
            if score > 0.95:
                break
        if best_mode is not None and len(best) >= MIN_POINTS and best_score > 0.8:
            self.mode = best_mode
            print(u'  [{}] quy ước toạ độ: filter {}, điểm {}'.format(
                self.inst.Name,
                u'cục bộ' if best_mode[0] else u'mô hình',
                u'áp Transform' if best_mode[1] else u'giữ nguyên'))
        return [p for p in best if inside_box(p, lo, hi)]


def query_all(sources, origin, axes, lo, hi, max_points=MAX_POINTS):
    pts = []
    for s in sources:
        pts.extend(s.points_in_box(origin, axes, lo, hi, max_points))
    return pts


# ----------------------- Hình học đối tượng -----------------------

def get_profile(el, cat_id):
    """('round', R, R) hoặc ('rect', nửa rộng, nửa cao), đơn vị feet."""
    if cat_id == int(BuiltInCategory.OST_PipeCurves):
        d = get_double(el, BuiltInParameter.RBS_PIPE_OUTER_DIAMETER)
        return ('round', d / 2, d / 2) if d else None
    if cat_id == int(BuiltInCategory.OST_Conduit):
        d = get_double(el, BuiltInParameter.RBS_CONDUIT_OUTER_DIAM_PARAM)
        return ('round', d / 2, d / 2) if d else None
    if cat_id == int(BuiltInCategory.OST_DuctCurves):
        d = get_double(el, BuiltInParameter.RBS_CURVE_DIAMETER_PARAM)
        if d:
            return ('round', d / 2, d / 2)
        w = get_double(el, BuiltInParameter.RBS_CURVE_WIDTH_PARAM)
        h = get_double(el, BuiltInParameter.RBS_CURVE_HEIGHT_PARAM)
        return ('rect', w / 2, h / 2) if w and h else None
    if cat_id == int(BuiltInCategory.OST_CableTray):
        w = get_double(el, BuiltInParameter.RBS_CABLETRAY_WIDTH_PARAM)
        h = get_double(el, BuiltInParameter.RBS_CABLETRAY_HEIGHT_PARAM)
        return ('rect', w / 2, h / 2) if w and h else None
    return None


def orthogonalize(v, d):
    v = v.Subtract(d.Multiply(v.DotProduct(d)))
    return v.Normalize() if v.GetLength() > 1e-9 else None


def cross_axes(el, d):
    """Trục chiều rộng (w) và chiều cao (h) của tiết diện."""
    w = None
    try:
        for c in el.ConnectorManager.Connectors:
            w = orthogonalize(c.CoordinateSystem.BasisX, d)
            break
    except Exception:
        w = None
    if w is None:
        if abs(d.Z) < 0.99:
            w = XYZ.BasisZ.CrossProduct(d).Normalize()
        else:
            w = orthogonalize(XYZ.BasisX, d)
    h = d.CrossProduct(w).Normalize()
    return w, h


def dev_round(u, v, R, _):
    return math.hypot(u, v) - R


def dev_rect(u, v, A, B):
    du, dv = abs(u) - A, abs(v) - B
    if du > 0 or dv > 0:
        return math.hypot(max(du, 0.0), max(dv, 0.0))
    return max(du, dv)  # âm: điểm nằm bên trong tiết diện


def fit_circle(uv):
    """Khớp đường tròn (phương pháp Kasa). Trả về (cu, cv, r) hoặc None."""
    n = len(uv)
    if n < 20:
        return None
    Suu = Suv = Svv = Su = Sv = Suz = Svz = Sz = 0.0
    for u, v in uv:
        z = u * u + v * v
        Suu += u * u; Suv += u * v; Svv += v * v
        Su += u; Sv += v
        Suz += u * z; Svz += v * z; Sz += z
    M = [[Suu, Suv, Su], [Suv, Svv, Sv], [Su, Sv, float(n)]]
    rhs = [Suz, Svz, Sz]

    def det3(m):
        return (m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
                m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
                m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]))

    D = det3(M)
    if abs(D) < 1e-18:
        return None
    sol = []
    for col in range(3):
        Mc = [row[:] for row in M]
        for r in range(3):
            Mc[r][col] = rhs[r]
        sol.append(det3(Mc) / D)
    cu, cv = sol[0] / 2.0, sol[1] / 2.0
    r2 = sol[2] + cu * cu + cv * cv
    if r2 <= 0:
        return None
    return cu, cv, math.sqrt(r2)


# --------------------------- Phân tích ----------------------------

def analyze_linear(el, profile, sources):
    loc = el.Location
    if not isinstance(loc, LocationCurve) or not isinstance(loc.Curve, Line):
        return None
    shape, A, B = profile
    p0, p1 = loc.Curve.GetEndPoint(0), loc.Curve.GetEndPoint(1)
    L = p0.DistanceTo(p1)
    if L < 1e-6:
        return None
    d = p1.Subtract(p0).Normalize()
    w, h = cross_axes(el, d)
    axes = (d, w, h)
    band, tol = to_ft(BAND_MM), to_ft(TOL_MM)
    ext = max(A, B) + band  # hộp vuông để không phụ thuộc hướng tiết diện

    # Truy vấn theo từng đoạn để giới hạn MAX_POINTS không làm lệch độ phủ
    pts = []
    chunk = to_ft(CHUNK_MM)
    n_chunks = max(1, int(math.ceil(L / chunk)))
    for i in range(n_chunks):
        t0, t1 = i * L / n_chunks, (i + 1) * L / n_chunks
        pts.extend(query_all(sources, p0, axes, (t0, -ext, -ext), (t1, ext, ext)))

    def evaluate(a, b):
        f = dev_round if shape == 'round' else dev_rect
        res = []
        for t, u, v in pts:
            dv = f(u, v, a, b)
            if abs(dv) <= band:
                res.append((t, u, v, dv))
        return res, median([abs(x[3]) for x in res])

    band_pts, med = evaluate(A, B)
    if shape == 'rect' and abs(A - B) > 1e-6:
        # Hướng rộng/cao lấy từ connector có thể bị đảo: chọn hướng khớp hơn
        alt_pts, alt_med = evaluate(B, A)
        if alt_med is not None and (med is None or alt_med < med):
            band_pts, med = alt_pts, alt_med

    row = {
        'length_m': L * FT_TO_MM / 1000.0,
        'n_band': len(band_pts),
        'n_near': 0, 'pct_near': None, 'median_dev_mm': None,
        'coverage_pct': None, 'fit_offset_mm': None, 'scan_diam_mm': None,
    }
    if len(band_pts) < MIN_POINTS:
        row['status'] = 'KHONG_CO_DIEM'
        return row

    near = [x for x in band_pts if abs(x[3]) <= tol]
    row['n_near'] = len(near)
    row['pct_near'] = 100.0 * len(near) / len(band_pts)
    row['median_dev_mm'] = to_mm(med)

    bin_len = to_ft(BIN_MM)
    n_bins = max(1, int(math.ceil(L / bin_len)))
    counts = [0] * n_bins
    for x in near:
        counts[min(n_bins - 1, max(0, int(x[0] / bin_len)))] += 1
    coverage = sum(1 for c in counts if c >= MIN_BIN_POINTS) / float(n_bins)
    row['coverage_pct'] = 100.0 * coverage

    fit = None
    if shape == 'round':
        fit = fit_circle([(x[1], x[2]) for x in band_pts])
        if fit:
            # Lọc lại quanh đường tròn vừa khớp để loại điểm nhiễu (trần, giá treo...)
            cu, cv, r = fit
            inliers = [(x[1], x[2]) for x in band_pts
                       if abs(math.hypot(x[1] - cu, x[2] - cv) - r) <= tol]
            fit = fit_circle(inliers)
        if fit and not (0.5 * A < fit[2] < 1.5 * A + tol):
            fit = None  # khớp không tin cậy (thường do chỉ quét được một cung nhỏ)
        if fit:
            row['fit_offset_mm'] = to_mm(math.hypot(fit[0], fit[1]))
            row['scan_diam_mm'] = to_mm(2 * fit[2])

    if fit and math.hypot(fit[0], fit[1]) > tol:
        row['status'] = 'LECH'
    elif fit and abs(fit[2] - A) > tol:
        row['status'] = 'SAI_KICH_THUOC'
    elif not fit and med > tol:
        row['status'] = 'LECH'
    elif coverage < COVERAGE_OK:
        row['status'] = 'THIEU_MOT_PHAN'
    else:
        row['status'] = 'OK'
    return row


def analyze_box(el, sources):
    bb = el.get_BoundingBox(None)
    if bb is None:
        return None
    m = to_ft(TOL_MM)
    size = bb.Max.Subtract(bb.Min)
    axes = (XYZ.BasisX, XYZ.BasisY, XYZ.BasisZ)
    pts = query_all(sources, bb.Min, axes, (-m, -m, -m),
                    (size.X + m, size.Y + m, size.Z + m), max_points=MIN_POINTS * 10)
    return {
        'n_band': len(pts),
        'status': 'CO_DIEM' if len(pts) >= MIN_POINTS else 'KHONG_CO_DIEM',
    }


def size_text(profile):
    if not profile:
        return ''
    shape, A, B = profile
    if shape == 'round':
        return u'Ø%.0f' % to_mm(2 * A)
    return u'%.0fx%.0f' % (to_mm(2 * A), to_mm(2 * B))


def type_text(doc, el):
    try:
        t = doc.GetElement(el.GetTypeId())
        return u'%s : %s' % (t.FamilyName, t.Name) if t else el.Name
    except Exception:
        return u''


# ----------------------------- Xuất -------------------------------

COLUMNS = [
    ('id', 'ElementId'), ('cat', u'Hạng mục'), ('type', u'Family : Type'),
    ('size', u'Kích thước (mm)'), ('length_m', u'Dài (m)'),
    ('n_band', u'Số điểm quanh đối tượng'), ('n_near', u'Số điểm khớp'),
    ('pct_near', u'% điểm khớp'), ('median_dev_mm', u'Lệch trung vị (mm)'),
    ('coverage_pct', u'Độ phủ (%)'), ('fit_offset_mm', u'Lệch tâm ống (mm)'),
    ('scan_diam_mm', u'ĐK đo từ scan (mm)'), ('status', u'Trạng thái'),
]


def csv_cell(v):
    if v is None:
        return ''
    if isinstance(v, float):
        return '%.1f' % v
    s = u'%s' % v
    if any(c in s for c in ',"\n'):
        s = '"' + s.replace('"', '""') + '"'
    return s


def write_csv(path, rows):
    sw = StreamWriter(path, False, UTF8Encoding(True))
    try:
        sw.WriteLine(u','.join(csv_cell(c[1]) for c in COLUMNS))
        for r in rows:
            sw.WriteLine(u','.join(csv_cell(r.get(c[0])) for c in COLUMNS))
    finally:
        sw.Close()


def solid_fill_id(doc):
    for fp in FilteredElementCollector(doc).OfClass(FillPatternElement):
        if fp.GetFillPattern().IsSolidFill:
            return fp.Id
    return ElementId.InvalidElementId


def colorize(doc, view, rows):
    fill_id = solid_fill_id(doc)
    t = Transaction(doc, u'Tô màu đối chiếu Scan to BIM')
    t.Start()
    try:
        for r in rows:
            c = STATUS_COLORS.get(r['status'])
            if c is None:
                continue
            ogs = OverrideGraphicSettings()
            ogs.SetProjectionLineColor(c)
            ogs.SetSurfaceForegroundPatternId(fill_id)
            ogs.SetSurfaceForegroundPatternColor(c)
            view.SetElementOverrides(ElementId(r['id']), ogs)
        t.Commit()
    except Exception as ex:
        t.RollBack()
        print(u'Không tô màu được (view có thể đang bị View Template khoá): %s' % ex)


# ------------------------------ Main ------------------------------

def main():
    uidoc = __revit__.ActiveUIDocument  # noqa: F821 (biến có sẵn trong pyRevit/RPS)
    doc = uidoc.Document
    view = doc.ActiveView

    try:
        from pyrevit import script
        output = script.get_output()
        link = lambda eid: output.linkify(ElementId(eid))
    except Exception:
        link = lambda eid: str(eid)

    clouds = list(FilteredElementCollector(doc).OfClass(PointCloudInstance))
    if not clouds:
        print(u'Không tìm thấy point cloud trong mô hình.\n'
              u'Hãy chèn file .rcs/.rcp bằng Insert > Point Cloud rồi chạy lại.')
        return
    sources = [CloudSource(c) for c in clouds]
    print(u'Point cloud: %s' % u', '.join(c.Name for c in clouds))

    sel_ids = list(uidoc.Selection.GetElementIds())
    if sel_ids:
        elements = [doc.GetElement(i) for i in sel_ids]
        print(u'Kiểm tra %d đối tượng đang chọn.' % len(elements))
    else:
        elements = list(FilteredElementCollector(doc, view.Id)
                        .WhereElementIsNotElementType())
        print(u'Không có đối tượng nào được chọn, kiểm tra toàn bộ view "%s".' % view.Name)

    targets = []
    for el in elements:
        if el is None or el.Category is None:
            continue
        cid = el.Category.Id.IntegerValue
        if cid in LINEAR_CATS or cid in BOX_CATS:
            targets.append((el, cid))
    print(u'Số đối tượng thuộc hạng mục hỗ trợ: %d\n' % len(targets))
    if not targets:
        return

    rows, skipped = [], 0
    start = time.time()
    for i, (el, cid) in enumerate(targets):
        if i and i % 50 == 0:
            print(u'  ... %d/%d (%.0f s)' % (i, len(targets), time.time() - start))
        profile = get_profile(el, cid) if cid in LINEAR_CATS else None
        try:
            if profile:
                row = analyze_linear(el, profile, sources)
            else:
                row = analyze_box(el, sources)
        except Exception as ex:
            print(u'  Lỗi ở %s: %s' % (el.Id.IntegerValue, ex))
            row = None
        if row is None:
            skipped += 1
            continue
        row.update({
            'id': el.Id.IntegerValue,
            'cat': LINEAR_CATS.get(cid) or BOX_CATS.get(cid),
            'type': type_text(doc, el),
            'size': size_text(profile),
        })
        rows.append(row)

    elapsed = time.time() - start

    # Tổng hợp
    summary = {}
    for r in rows:
        summary[r['status']] = summary.get(r['status'], 0) + 1
    print(u'\n===== KẾT QUẢ (%d đối tượng, %.0f giây) =====' % (len(rows), elapsed))
    for st in ('OK', 'CO_DIEM', 'LECH', 'SAI_KICH_THUOC', 'THIEU_MOT_PHAN', 'KHONG_CO_DIEM'):
        if st in summary:
            print(u'  %-15s %6d  (%.0f%%)' % (st, summary[st], 100.0 * summary[st] / len(rows)))
    if skipped:
        print(u'  Bỏ qua: %d (ống cong hoặc không có hình học)' % skipped)

    if len(rows) >= 20 and summary.get('KHONG_CO_DIEM', 0) > 0.7 * len(rows):
        print(u'\n!! Hơn 70% đối tượng không có điểm quét: có thể point cloud chưa căn '
              u'đúng vị trí với mô hình. Kiểm tra lại cách chèn (Origin to Origin / '
              u'Shared Coordinates) trước khi đọc kết quả.')

    worst = sorted([r for r in rows if r['status'] in ('LECH', 'SAI_KICH_THUOC')],
                   key=lambda r: -(r.get('fit_offset_mm') or r.get('median_dev_mm') or 0))
    if worst:
        print(u'\nTop đối tượng lệch nhiều nhất:')
        for r in worst[:20]:
            print(u'  %s  %s %s  lệch %.0f mm  %s' % (
                link(r['id']), r['cat'], r['size'],
                r.get('fit_offset_mm') or r.get('median_dev_mm') or 0,
                u'(ĐK scan %.0f mm)' % r['scan_diam_mm'] if r.get('scan_diam_mm') else u''))

    folder = os.path.dirname(doc.PathName) if doc.PathName else os.path.expanduser('~')
    name = os.path.splitext(os.path.basename(doc.PathName or 'model'))[0]
    stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M')
    csv_path = os.path.join(folder, u'DoiChieu_%s_%s.csv' % (name, stamp))
    write_csv(csv_path, rows)
    print(u'\nĐã xuất báo cáo: %s' % csv_path)

    if COLORIZE:
        colorize(doc, view, rows)
        print(u'Đã tô màu trong view "%s".' % view.Name)


main()
