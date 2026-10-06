# Faz 2 statik test tabloları

> OTOMATİK ÜRETİLDİ: `scripts/static_tests.sh` + `python3 scripts/analyze_static.py`. Tahminler `config/vehicle_derived.yaml`'dan.

## Koşular, zaman tabanı ve itki temizliği

Sayım oranı = mesaj sayısı / (rate × kayıt süresi + 1); 1 = düşme yok. İtki/tork max: ThrusterState'te kaydedilen en büyük |değer| (sıfır setpoint'e rağmen; bkz. SimpleThruster bulgusu). İlk durum: ilk kaydedilen DebugPhysics mesajında max |açısal hız| ve |u|, |v|.

| Koşu | Debug N / oran | Odometri N / oran | max \|itki\| [N] / \|tork\| [N·m] | İlk \|ω\| [rad/s] / \|u,v\| [m/s] |
|---|---|---|---|---|
| depth_release | 2838 / 0.9998 | 1419 / 0.9998 | 0 / 0 | 4.8e-09 / 4.6e-13 |
| depth_default | 2338 / 0.9997 | 1172 / 0.9996 | 0 / 0 | 3.5e-05 / 3e-08 |
| surface | 9335 / 1.0000 | 4672 / 1.0000 | 0 / 0 | 0.004 / 5.7e-06 |
| surface_pitch | 4337 / 0.9998 | 2169 / 0.9998 | 0 / 0 | 0.1 / 0.00019 |
| roll_neutral | 3338 / 1.0000 | 1673 / 0.9999 | 0 / 0 | 0.91 / 0.00011 |
| pitch_neutral | 4806 / 1.0000 | 2406 / 0.9999 | 0 / 0 | 0.012 / 4.1e-07 |
| roll_neutral_500 | 16539 / 1.0000 | 1653 / 1.0000 | 0 / 0 | 0.64 / 4.1e-05 |
| roll_neutral_50 | 1668 / 0.9995 | 1671 / 0.9995 | 0 / 0 | 0.82 / 8.1e-05 |
| surface_roll | 4303 / 0.9998 | 2155 / 0.9998 | 0 / 0 | 0.43 / 5.4e-05 |

## S1. Yükleme ve tutarlılık (depth_release, ilk mesaj)

| Büyüklük | Ölçülen | Tahmin | Fark | Tolerans | Sonuç |
|---|---|---|---|---|---|
| kütle m [kg] | 17.6377 | 17.6377 | -1.25e-10 | 1e-06 | PASS |
| hacim V [m³] | 0.0185196 | 0.0185196 | +2.79e-09 | 1e-06 | PASS |
| yüzey S [m²] | 0.527396 | 0.527396 | -3.59e-09 | 1e-06 | PASS |
| kaldırma F_b,z [N] (dünya) | -181.677 | -181.677 | +2.79e-09 | 1e-06 | PASS |
| atalet I_xx [kg·m²] | 0.0434849 | 0.0434849 | +1.02e-10 | 1e-05 | PASS |
| atalet I_yy [kg·m²] | 1.22584 | 1.22584 | -2.34e-10 | 1e-05 | PASS |
| atalet I_zz [kg·m²] | 1.21581 | 1.21581 | -2.34e-10 | 1e-05 | PASS |
| CG [m] (max |fark|) | (0.0232596, 0, 0.015) | (0.0232596, -2.83806e-18, 0.015) | +1.98e-12 | 1e-06 | PASS |
| C_d,x | 0.0632341 | 0.0632341 | +2.81e-11 | 1e-06 | PASS |
| C_f,x [m/s] | 0.00132863 | 0.00132863 | -2.89e-11 | 1e-06 | PASS |
| C_d,y | 1.77778 | 1.77778 | +1.25e-10 | 1e-06 | PASS |
| C_f,y [m/s] | 0.0217525 | 0.0217525 | +2.15e-10 | 1e-06 | PASS |
| C_d,z | 1.77778 | 1.77778 | +1.25e-10 | 1e-06 | PASS |
| C_f,z [m/s] | 0.0217525 | 0.0217525 | +2.15e-10 | 1e-06 | PASS |
| basınç / ρ g z (ilk 20 örnek, max |fark|) | 49029.5 | 49029.1 | +4.67e-05 | 0.001 | PASS |

## S2. Varsayılan hidrodinamik (depth_default)

| Büyüklük | Ölçülen | Tahmin | Fark | Tolerans | Sonuç |
|---|---|---|---|---|---|
| varsayılan C_d,x (MVAE portu) | 0.125 | 0.125 | +0 | 0.005 | PASS |
| varsayılan C_f,x [m/s] (= 0.1 C_d) | 0.0125 | 0.0125 | +0 | 0.005 | PASS |
| varsayılan C_d,y (MVAE portu) | 1 | 1 | +0 | 0.005 | PASS |
| varsayılan C_f,y [m/s] (= 0.1 C_d) | 0.1 | 0.1 | +0 | 0.005 | PASS |
| varsayılan C_d,z (MVAE portu) | 1 | 1 | +0 | 0.005 | PASS |
| varsayılan C_f,z [m/s] (= 0.1 C_d) | 0.1 | 0.1 | +0 | 0.005 | PASS |
| M_eff [kg] (override'dan bağımsız) | 27.2189 | 27.2188 | +5.17e-06 | 0.01 | PASS |
| M_eff (varsayılan) / M_eff (kalibre) | 0.999998 | 1 | -2.35e-06 | 0.005 | PASS |
| terminal w∞ [m/s] (varsayılan C_d/C_f) | 0.263277 | 0.263735 | -0.00174 | 0.005 | PASS |
| F_p / F_f (vektör) / yüz bazlı port, max göreli fark | 2.92641e-05 | 0 | +2.97e-05 | 0.001 | PASS |

## S3. Serbest yükselme (depth_release)

| Büyüklük | Ölçülen | Tahmin | Fark | Tolerans | Sonuç |
|---|---|---|---|---|---|
| M_eff [kg] (F_net = M·ẇ) | 27.219 | 27.2188 | +7.52e-06 | 0.01 | PASS |
| terminal yükselme hızı w∞ [m/s] | 0.354505 | 0.354706 | -0.000567 | 0.005 | PASS |
| F_p (vektör) / yüz bazlı port, max göreli fark | 0.000465195 | 0 | +0.000465 | 0.001 | PASS |
| F_f (vektör) / yüz bazlı port, max göreli fark | 0.000174542 | 0 | +0.000175 | 0.001 | PASS |
| F_p,z / (½ρC_d,z A_plan w³), ortalama (bilgi) | 1.00056 | 1 | +0.000556 | — | bilgi |
| F_f,z / (ρ C_f,z S_t,z w), ortalama (bilgi) | 1.0015 | 1 | +0.0015 | — | bilgi |
| fiziksel Δt / nominal (max) | 1.00001 | 1 | +7.52e-06 | 0.5 | PASS |

## S4. Yüzey dengesi (surface, surface_pitch, surface_roll)

| Büyüklük | Ölçülen | Tahmin | Fark | Tolerans | Sonuç |
|---|---|---|---|---|---|
| eksen (orijin) derinliği [cm], son 30 s | 5.93539 | 5.93322 | +0.00216 | 0.05 | PASS |
| trim [°], son 30 s | -0.167182 | -0.167802 | +0.00062 | 0.05 | PASS |
| roll [°], son 30 s | 4.73562e-07 | 0 | +4.74e-07 | 0.05 | PASS |
| V_sub [m³] = m/ρ, son 30 s | 0.0176382 | 0.0176377 | +3e-05 | 0.001 | PASS |
| yüzey heave periyodu T_n [s] (bilgi) | 1.1242 | 1.11366 | +0.00947 | — | bilgi |
| yüzey pitch periyodu T_n [s] (bilgi; GM_L ±0.05°) | 0.93644 | 0.929986 | +0.00694 | — | bilgi |
| yüzey pitch periyodu T_n [s] (bilgi; GM_L 2° sekant) | 0.93644 | 0.997322 | -0.061 | — | bilgi |
| yüzey roll periyodu T_n [s] (bilgi; I_xx, W·GM_T) | 0.812847 | 0.813468 | -0.000764 | — | bilgi |
| yüzey roll sönümü σ [1/s] / model (c DebugPhysics'ten) | 0.0654459 | 0.0758755 | -0.0104 | 0.02 | PASS |
| yüzey roll max |açı| [°]: ilk 1 s → son 5 s (bilgi) | 4.91058 | 0.000665604 | — | — | bilgi |

## S5. Dalmış salınım, nötr varyant (roll_neutral[_500, _50], pitch_neutral)

| Büyüklük | Ölçülen | Tahmin | Fark | Tolerans | Sonuç |
|---|---|---|---|---|---|
| roll doğal periyodu T_n [s] (I + aI) | 0.796183 | 0.793695 | +0.00314 | 0.01 | PASS |
| roll T_n / T_n(ek atalet yok) | 1.00314 | 1 | +0.00314 | 0.01 | PASS |
| roll ek atalet aI [kg·m²] | 0.000273095 | 0 | +0.000273 | 0.00087 | PASS |
| roll kaldırma momenti / (B·BG·sinθ) (gecikme taramalı) | 1.0009 | 1 | +0.000901 | 0.001 | PASS |
| roll sönüm σ [1/s] (− = büyüyen) / model | -0.107271 | -0.110911 | +0.00364 | 0.02 | PASS |
| roll max |açı| [°]: ilk 1 s → son 5 s (bilgi) | 10.8802 | 65.4186 | — | — | bilgi |
| pitch doğal periyodu T_n [s] (I + aI) | 4.71705 | 4.72931 | -0.00259 | 0.02 | PASS |
| pitch T_n / T_n(ek atalet yok) | 1.11936 | 1.12227 | -0.00259 | 0.02 | PASS |
| pitch ek atalet aI [kg·m²] | 0.310098 | 0.318086 | -0.00799 | 0.062 | PASS |
| pitch kaldırma momenti / (B·BG·sinθ) (gecikme taramalı) | 1.00024 | 1 | +0.000237 | 0.001 | PASS |
| pitch sönüm σ [1/s] (− = büyüyen) / model | 0.229014 | 0.234809 | -0.00579 | 0.02 | PASS |
| pitch max |açı| [°]: ilk 1 s → son 5 s (bilgi) | 4.99736 | 0.00266894 | — | — | bilgi |
| roll T_n [s], 500 Hz sim | 0.799392 | 0.793695 | +0.00718 | 0.01 | PASS |
| roll sönüm σ [1/s], 500 Hz sim / model | -0.228233 | -0.236249 | +0.00802 | 0.02 | PASS |
| roll max |açı| [°], 500 Hz: ilk 1 s → son 5 s (bilgi) | 12.0303 | 105.18 | — | — | bilgi |
| roll T_n [s], 50 Hz sim | 0.793779 | 0.793695 | +0.000106 | 0.01 | PASS |
| roll sönüm σ [1/s], 50 Hz sim / model | 0.0471825 | 0.045761 | +0.00142 | 0.02 | PASS |
| roll max |açı| [°], 50 Hz: ilk 1 s → son 5 s (bilgi) | 9.83081 | 2.65689 | — | — | bilgi |

S3 ayrıntı: M_eff fitinde 185 örnek (|F_net| > 1 N), kuvvet kayması 1, artık std 0.000178 N. Fiziksel Δt / nominal: 0.999 … 1.000. Yüz bazlı port kıyası: 150 kuvvet güncellemesi, hız kayması 0 mesaj, medyan göreli fark F_p 1.9e-05 / F_f 1.9e-05. Yükselmede max |pitch| 1.41°, max |u| 0.00716 m/s. Fiziksel heave M (m + m_a,z, Lamb) = 35.13 kg; ek kütlesiz m = 17.64 kg.

Yüzeye çıkış (bilgi, Faz 6): eksen derinliği min 0.78 cm (negatif = eksen su üstünde), pitch aralığı -2.77° … 6.26°.

S4 ayrıntı: dik (trimsiz) denge eksen derinliği tahmini 5.943 cm; trim dahil 5.933 cm (orijin). Son 30 s z std 0.930 mm. A_wp = 883.2 cm², GM_L (±0.05°) = 40.73 cm, GM_L (2° sekant) = 35.42 cm. Heave fiti: ζ = 0.005, göreli rms 0.052. Pitch fiti: ζ = 0.017, göreli rms 0.042. GM_T (yüzey, ±0.05°) = 1.499 cm. Yüzey roll fiti: σ = +0.0654 1/s (model +0.0759), göreli rms 0.019. Yüzeyde roll sönüm katsayısı c = 0.01957 N·m·s (dalmışta port 0.00398); sürtünme torku yönünden C_eff / C_f,x medyanı 8.22.

S5 roll: ω_d = 7.8909 rad/s, σ = -0.1073 1/s, ζ = -0.0136, sıfır geçiş periyodu 0.7963 s (5 geçiş), göreli rms 0.003. I_eff = B·BG/ω_n² = 0.0438 kg·m²; moment fitinde gecikme 15 ms. Sönüm modeli: c = 0.00398 N·m·s (port), σ_fiz = +0.0458, σ_gecikme = −0.1567 (P = 2, τ = 5.0 ms) → σ = -0.1109 1/s.

S5 pitch: ω_d = 1.3122 rad/s, σ = 0.2290 1/s, ζ = 0.1719, sıfır geçiş periyodu 4.7813 s (8 geçiş), göreli rms 0.001. I_eff = B·BG/ω_n² = 1.5359 kg·m²; moment fitinde gecikme 15 ms. Sönüm modeli: c = 0.7387 N·m·s (port), σ_fiz = +0.2392, σ_gecikme = −0.0044 (P = 2, τ = 5.0 ms) → σ = +0.2348 1/s.

S5 roll_500: ω_d = 7.8566 rad/s, σ = -0.2282 1/s, ζ = -0.0290, sıfır geçiş periyodu 0.7996 s (5 geçiş), göreli rms 0.013. Sönüm modeli: c = 0.00398 N·m·s (port), σ_fiz = +0.0458, σ_gecikme = −0.2820 (P = 10, τ = 9.0 ms) → σ = -0.2362 1/s.

S5 roll_50: ω_d = 7.9154 rad/s, σ = 0.0472 1/s, ζ = 0.0060, sıfır geçiş periyodu 0.7938 s (5 geçiş), göreli rms 0.000. Sönüm modeli: c = 0.00398 N·m·s (port), σ_fiz = +0.0458, σ_gecikme = −0.0000 (P = 1, τ = 0.0 ms) → σ = +0.0458 1/s.

## Faz 3 öngörüsü: surge direnci ve hücum açısı (yüz bazlı port, kalibre C_d/C_f)

Akış gövdeye göre α kadar eğik. X: gövde x direnci; artış α = 0'a göre. Port S2/S3'te DebugPhysics ile doğrulandı (medyan 2e-5); bu tablo canlı surge testi değil (Faz 3).

| u [m/s] | α [°] | X [N] | F_p,x / F_f,x [N] | Artış | Z [N] |
|---|---|---|---|---|---|
| 0.5 | 0.00 | 0.4111 | 0.0697 / 0.3413 | +0.0% | +0.0000 |
| 0.5 | 0.25 | 0.4346 | 0.0805 / 0.3542 | +5.7% | +0.0012 |
| 0.5 | 0.50 | 0.4587 | 0.0917 / 0.3670 | +11.6% | +0.0025 |
| 0.5 | 1.00 | 0.5082 | 0.1156 / 0.3926 | +23.6% | +0.0056 |
| 0.5 | 2.00 | 0.6132 | 0.1694 / 0.4437 | +49.2% | +0.0141 |
| 0.5 | 5.00 | 0.9762 | 0.3804 / 0.5958 | +137.5% | +0.0607 |
| 1.0 | 0.00 | 1.2405 | 0.5578 / 0.6827 | +0.0% | +0.0000 |
| 1.0 | 0.25 | 1.3520 | 0.6437 / 0.7084 | +9.0% | +0.0044 |
| 1.0 | 0.50 | 1.4673 | 0.7333 / 0.7340 | +18.3% | +0.0098 |
| 1.0 | 1.00 | 1.7099 | 0.9246 / 0.7852 | +37.8% | +0.0234 |
| 1.0 | 2.00 | 2.2430 | 1.3556 / 0.8875 | +80.8% | +0.0637 |
| 1.0 | 5.00 | 4.2346 | 3.0430 / 1.1916 | +241.4% | +0.3211 |
| 1.5 | 0.00 | 2.9067 | 1.8827 / 1.0240 | +0.0% | -0.0000 |
| 1.5 | 0.25 | 3.2349 | 2.1723 / 1.0625 | +11.3% | +0.0119 |
| 1.5 | 0.50 | 3.5758 | 2.4748 / 1.1010 | +23.0% | +0.0267 |
| 1.5 | 1.00 | 4.2985 | 3.1206 / 1.1779 | +47.9% | +0.0653 |
| 1.5 | 2.00 | 5.9063 | 4.5751 / 1.3312 | +103.2% | +0.1842 |
| 1.5 | 5.00 | 12.0574 | 10.2700 / 1.7874 | +314.8% | +0.9808 |

**Özet:** 49 PASS, 0 FAIL, 11 bilgi satırı.

