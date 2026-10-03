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
| Kütle m | 18.156 kg |
| Ağırlık W | 178.11 N |
| Kaldırma (tam dalmış) B | 181.68 N |
| Net yüzdürme B−W | +3.562 N (2.00% W) |
| Gövde (homojen) / balast | 13.156 kg / 5.000 kg |
| Balast konumu (x, z) | (2.33, 5.45) cm, gövde içinde: True |
| CB (gövde) | (2.326, 0, 0) cm |
| CG (gövde) | (2.326, 0, 1.500) cm  → BG = 1.50 cm |
| Atalet (CG, asal) Ixx, Iyy, Izz | 0.0456, 1.2765, 1.2657 kg·m² (köşegen: True) |

## T3. Yüzey dengesi ve stabilite

| Büyüklük | Değer |
|---|---|
| Eksen derinliği (dik, NED) | 6.619 cm |
| Su çekimi T | 14.119 cm |
| Fribord (silindir üstü) | 8.81 mm |
| Su altı hacim oranı | 98.04% |
| Statik trim (yüzey) | -0.132° (+ = burun yukarı) |
| Su hattı L_wl / B_wl / A_wp | 0.961 m / 7.06 cm / 646.0 cm² |
| KB / KG | 7.361 cm / 6.000 cm |
| BM_T / BM_L | 0.141 cm / 25.08 cm |
| GM_T yüzey: formül / sayısal (GZ eğimi, ±0.05°) | 1.502 / 1.501 cm |
| GM_L yüzey: formül / sayısal (±0.05°) | 26.44 / 26.35 cm |
| GM su altı (roll = pitch), sayısal | 1.530 cm (BG×B/W = 1.530) |
| Roll periyodu (su altı, ek atalet yok) | 0.81 s |
| Pitch periyodu (su altı, ek atalet yok) | 4.30 s |

## T4. Thruster yerleşimi

| Ad | Konum xyz [m] | Yön | rpy [rad] | Yüzey dengesinde derinlik |
|---|---|---|---|---|
| ThrusterPort | (-0.5500, -0.100, +0.000) | (1, 0, 0) | (0.0000, 0.0000, 0.0000) | 6.62 cm |
| ThrusterStarboard | (-0.5500, +0.100, +0.000) | (1, 0, 0) | (0.0000, 0.0000, 0.0000) | 6.62 cm |
| ThrusterHeaveBow | (+0.3033, +0.000, +0.000) | (0, 0, 1) | (0.0000, -1.5708, 0.0000) | 6.62 cm |
| ThrusterHeaveStern | (-0.2567, +0.000, +0.000) | (0, 0, 1) | (0.0000, -1.5708, 0.0000) | 6.62 cm |

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

Form faktörü (Hoerner): 1+k = 1.0800. Stonefish kalibre: C_d,x = 0.07703, C_f,x = 0.001036 m/s (bant [0.3, 1.5] m/s, max göreli hata 13.4%). Stonefish varsayılan (tahmini MVAE): C_d,x = 0.1250, C_f,x = 0.01250.

| u [m/s] | Re | C_F | R_sürt. [N] | R_bas. [N] | R_top. [N] | SF kalibre F_f / F_p [N] | SF kalibre top. [N] | Hata | SF varsayılan top. [N] |
|---|---|---|---|---|---|---|---|---|---|
| 0.3 | 3.16e+05 | 0.00612 | 0.1453 | 0.0116 | 0.1569 | 0.1596 / 0.0183 | 0.1780 | +13.4% | 1.96 |
| 0.5 | 5.27e+05 | 0.00541 | 0.3569 | 0.0285 | 0.3854 | 0.2661 / 0.0849 | 0.3510 | -8.9% | 3.35 |
| 1.0 | 1.05e+06 | 0.00463 | 1.2220 | 0.0977 | 1.3197 | 0.5321 / 0.6795 | 1.2116 | -8.2% | 7.53 |
| 1.5 | 1.58e+06 | 0.00425 | 2.5237 | 0.2018 | 2.7255 | 0.7982 / 2.2933 | 3.0915 | +13.4% | 13.36 |
| 2.0 | 2.11e+06 | 0.00401 | 4.2310 | 0.3383 | 4.5693 | 1.0642 / 5.4360 | 6.5002 | +42.3% | 21.67 |

Heave (y = z): çapraz akış C_D,c = 0.8 (varsayım), A_plan = 0.1655 m². Kalibre: C_d,z = 1.7778, C_f,z = 0.02175 m/s (bant [0.1, 0.4] m/s, max göreli hata 11.1%).

Tam surge itkisi 20 N ile terminal hız: fiziksel (çıplak gövde) 4.53 m/s, Stonefish kalibre 3.00 m/s (bant dışı). Yüzeyden dalış + w = 0.3 m/s için gereken dikey itki ≈ 9.5 N (kapasite 20 N).

## T6. Ek kütle

| | Eksenel (x) | Yanal (y, z) | Surge yönünde etkin atalet |
|---|---|---|---|
| Fiziksel (prolate, L/D=8, Lamb) | k₁ = 0.0293 → 0.542 kg | k₂ = 0.9447 → 17.496 kg | m + m_a,x = 18.70 kg |
| Stonefish (tahmini MVAE yarı-eksenleri [0.6, 0.075, 0.075]) | 0.469 kg | 14.137 kg | m + ort(m_a) = 27.74 kg |

## T7. VBS (opsiyonel, şu an kapalı)

Max 0.60 L → net yüzdürme +3.56 N ile -2.32 N arası (Stonefish VBS sadece ağırlık ekler, ataleti değiştirmez).

## T8. Çapraz kontroller

| Kontrol | Sonuç |
|---|---|
| Mesh kapalı + tutarlı yönlü, Euler = 2 | True, 2 |
| Dilimleme (tam dalmış) V / mesh V − 1 | +5.86e-07 |
| Dilimleme CB_x − mesh CB_x | +0.0001 mm |
| GM_T yüzey (sayısal) / BG (dairesel kesit → metasantr eksende) | 1.0008 |
| GM_T formül / sayısal | 1.0002 |
| Balast gövde içinde | True |
| Atalet tensörü köşegen | True |
| Allocation rank (X, Z, M, N) | 4 |
| Tüm thrusterlar yüzey dengesinde su altında | True |

