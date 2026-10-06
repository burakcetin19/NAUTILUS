# Faz 1 tasarım tabloları

> OTOMATİK ÜRETİLDİ: `python3 scripts/design_vehicle.py`. Girdi: `config/vehicle.yaml`.

## T1. Geometri

| Büyüklük | Analitik | Mesh-tam | Fark |
|---|---|---|---|
| Boy L / çap D / L/D | 1.200 m / 0.150 m / 8.00 | | |
| Burun / silindir / kıç | 0.15 / 0.75 / 0.3 m | | |
| Hacim V [L] | 18.5550 | 18.5196 | -0.191% |
| Islak alan S [m²] | 0.5277 | 0.5274 | -0.064% |
| Ön kesit A_ön [cm²] | 176.7146 | 176.4309 | -0.161% |
| Plan alanı A_plan [m²] | 0.1655 | 0.1655 | -0.023% |
| S_t (surge teğetsel, ΣA(1−n_x²)) [m²] | | 0.5138 | |
| S_t,z (heave teğetsel) [m²] | | 0.2705 | |
| x_CB [cm] | 2.321 | 2.326 | |
| Mesh | 3970 köşe, 7936 üçgen, watertight=True, Euler=2 | | |

## T2. Kütle ve yüzdürme

| Büyüklük | Değer |
|---|---|
| Kütle m | 17.638 kg |
| Ağırlık W | 173.03 N |
| Kaldırma (tam dalmış) B | 181.68 N |
| Net yüzdürme B−W | +8.651 N (5.00% W) |
| Gövde (homojen) / balast | 12.638 kg / 5.000 kg |
| Balast konumu (x, z) | (2.33, 5.29) cm, gövde içinde: True |
| CB (gövde) | (2.326, 0, 0) cm |
| CG (gövde) | (2.326, 0, 1.500) cm  → BG = 1.50 cm |
| Atalet (CG, asal) Ixx, Iyy, Izz | 0.0435, 1.2258, 1.2158 kg·m² (köşegen: True) |

## T3. Yüzey dengesi ve stabilite

| Büyüklük | Değer |
|---|---|
| Eksen derinliği (dik, NED) | 5.943 cm |
| Su çekimi T | 13.443 cm |
| Fribord (silindir üstü) | 15.57 mm |
| Su altı hacim oranı | 95.24% |
| Statik trim (yüzey) | -0.168° (+ = burun yukarı) |
| Su hattı L_wl / B_wl / A_wp | 1.024 m / 9.15 cm / 883.2 cm² |
| KB / KG | 7.172 cm / 6.000 cm |
| BM_T / BM_L | 0.330 cm / 39.39 cm |
| GM_T yüzey: formül / sayısal (GZ eğimi, ±0.05°) | 1.502 / 1.499 cm |
| GM_L yüzey: formül / sayısal (±0.05°) | 40.56 / 40.42 cm |
| GM su altı (roll = pitch), sayısal | 1.575 cm (BG×B/W = 1.575) |
| Roll periyodu (su altı, ek atalet yok) | 0.79 s |
| Pitch periyodu (su altı, ek atalet yok) | 4.21 s |
| Roll / pitch periyodu (su altı, Stonefish I + aI) | 0.79 / 4.73 s |

## T4. Thruster yerleşimi

| Ad | Konum xyz [m] | Yön | rpy [rad] | Yüzey dengesinde derinlik |
|---|---|---|---|---|
| ThrusterPort | (-0.5500, -0.100, +0.000) | (1, 0, 0) | (0.0000, 0.0000, 0.0000) | 5.94 cm |
| ThrusterStarboard | (-0.5500, +0.100, +0.000) | (1, 0, 0) | (0.0000, 0.0000, 0.0000) | 5.94 cm |
| ThrusterHeaveBow | (+0.3033, +0.000, +0.000) | (0, 0, 1) | (0.0000, -1.5708, 0.0000) | 5.94 cm |
| ThrusterHeaveStern | (-0.2567, +0.000, +0.000) | (0, 0, 1) | (0.0000, -1.5708, 0.0000) | 5.94 cm |

Allocation matrisi B (τ = B·T, CG etrafında; T_max = ±10.0 N, rank = 4):

| Eksen | ThrusterPort | ThrusterStarboard | ThrusterHeaveBow | ThrusterHeaveStern | Kapasite |
|---|---|---|---|---|---|
| X | +1.0000 | +1.0000 | +0.0000 | +0.0000 | 20.00 N |
| Y | +0.0000 | +0.0000 | +0.0000 | +0.0000 | 0.00 N |
| Z | +0.0000 | +0.0000 | +1.0000 | +1.0000 | 20.00 N |
| K | +0.0000 | +0.0000 | +0.0000 | +0.0000 | 0.00 N·m |
| M | -0.0150 | -0.0150 | -0.2800 | +0.2800 | 5.90 N·m |
| N | +0.1000 | -0.1000 | +0.0000 | +0.0000 | 2.00 N·m |

## T5. Surge direnci

Form faktörü (Hoerner): 1+k = 1.0800. Stonefish kalibre: C_d,x = 0.06323, C_f,x = 0.001329 m/s (bant [0.5, 1.5] m/s, max göreli hata 6.6%). Stonefish varsayılan (MVAE portu): C_d,x = 0.1250, C_f,x = 0.01250.

| u [m/s] | Re | C_F | R_sürt. [N] | R_bas. [N] | R_top. [N] | SF kalibre F_f / F_p [N] | SF kalibre top. [N] | Hata | SF varsayılan top. [N] |
|---|---|---|---|---|---|---|---|---|---|
| 0.3 | 3.16e+05 | 0.00612 | 0.1453 | 0.0116 | 0.1569 | 0.2048 / 0.0151 | 0.2199 | +40.1% | 1.96 |
| 0.4 | 4.22e+05 | 0.00571 | 0.2408 | 0.0193 | 0.2601 | 0.2731 / 0.0357 | 0.3088 | +18.7% | 2.64 |
| 0.5 | 5.27e+05 | 0.00541 | 0.3569 | 0.0285 | 0.3854 | 0.3413 / 0.0697 | 0.4111 | +6.6% | 3.35 |
| 1.0 | 1.05e+06 | 0.00463 | 1.2220 | 0.0977 | 1.3197 | 0.6827 / 0.5578 | 1.2405 | -6.0% | 7.53 |
| 1.5 | 1.58e+06 | 0.00425 | 2.5237 | 0.2018 | 2.7255 | 1.0240 / 1.8827 | 2.9067 | +6.6% | 13.36 |
| 2.0 | 2.11e+06 | 0.00401 | 4.2310 | 0.3383 | 4.5693 | 1.3654 / 4.4626 | 5.8280 | +27.5% | 21.67 |

Heave (y = z): çapraz akış C_D,c = 0.8 (varsayım), A_plan = 0.1655 m². Kalibre: C_d,z = 1.7778, C_f,z = 0.02175 m/s (bant [0.1, 0.4] m/s, max göreli hata 11.1%).

Tam surge itkisi 20 N ile terminal hız: fiziksel (çıplak gövde) 4.53 m/s, Stonefish kalibre 3.17 m/s (bant dışı). Yüzeyden dalış + w = 0.3 m/s için gereken dikey itki ≈ 14.6 N (kapasite 20 N).

## T6. Ek kütle

| | Eksenel (x) | Yanal (y, z) | Surge yönünde etkin atalet |
|---|---|---|---|
| Fiziksel (prolate, L/D=8, Lamb) | k₁ = 0.0293 → 0.542 kg | k₂ = 0.9447 → 17.496 kg | m + m_a,x = 18.18 kg |
| Stonefish (MVAE portu: yarı-eksenler [0.6, 0.075, 0.075] m) | 0.469 kg | 14.137 kg | m + ort(m_a) = 27.22 kg |

MVAE portu (`hybrid_vehicle_sim/mvae.py`): Stonefish yükleyicisinin gördüğü 14208 köşe (kopyalar dahil), 0 iterasyon (k = 0 → sınırlayıcı kutu yarı-boyutları). Ek atalet (Stonefish, I + aI): aI = (0.0000, 0.3181, 0.3181) kg·m² (roll için kodda 0). Faz 2'de canlı ölçülüyor.

## T7. VBS (opsiyonel, şu an kapalı)

Max 0.60 L → net yüzdürme +8.65 N ile +2.77 N arası (Stonefish VBS sadece ağırlık ekler, ataleti değiştirmez). Nötr için gereken su hacmi 0.88 L; mevcut boyutla nötre ulaşılabilir mi: False.

## T8. Çapraz kontroller

| Kontrol | Sonuç |
|---|---|
| Mesh kapalı + tutarlı yönlü, Euler = 2 | True, 2 |
| Dilimleme (tam dalmış) V / mesh V − 1 | +5.86e-07 |
| Dilimleme CB_x − mesh CB_x | +0.0001 mm |
| GM_T yüzey (sayısal) / BG (dairesel kesit → metasantr eksende) | 0.9996 |
| GM_T formül / sayısal | 1.0019 |
| Balast gövde içinde | True |
| Atalet tensörü köşegen | True |
| Allocation rank (X, Z, M, N) | 4 |
| Tüm thrusterlar yüzey dengesinde su altında | True |

