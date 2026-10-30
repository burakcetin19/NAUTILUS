# Faz 1: Araç tasarım parametreleri (onaylandı, 2026-10-06)

Tarih: 2026-10-04, kararlar 2026-10-06. Bu fazda simülasyon çalıştırılmadı. Tüm değerler `config/vehicle.yaml` girdisinden `scripts/design_vehicle.py` ile hesaplandı. Tam tablolar otomatik olarak [`01_tasarim_tablolar.md`](01_tasarim_tablolar.md) dosyasına yazılıyor. Mesh'in Stonefish'te yüklenmesi ve ölçümler Faz 2'de: [`02_statik.md`](02_statik.md).

```bash
cd ~/stonefish_ws/src/hybrid_vehicle_sim
python3 scripts/design_vehicle.py          # ~30 s
```

Çıktılar:
- `meshes/hull.obj|json`: tek parça, watertight gövde. `meshes/thruster.obj`: thruster görseli (yalnızca görsel).
- `config/vehicle_derived.yaml`: senaryo üretecinin (`scripts/gen_scenario.py`) girdisi.
- `docs/01_tasarim_tablolar.md`
- `figures/faz1/*`

Kod `hybrid_vehicle_sim/` paketinde: `hull`, `hydrostatics`, `mass`, `drag`, `thrusters`, `mvae` (Stonefish elipsoid yaklaşımının portu, Faz 2). Faz 2–4'te yeniden kullanılıyor.

## 0. Kararlar (2026-10-06)

| # | Soru | Karar | Etkisi |
|---|---|---|---|
| 1 | Ölçek ve kütle | Ölçek aynı (L = 1.2 m, D = 0.15 m, BG = 1.5 cm). **Net yüzdürme %2 → %5** | m 18.16 → **17.64 kg**; B − W 3.56 → **8.65 N**; fribord 8.8 → **15.6 mm** |
| 2 | Thruster yerleşimi ve ±10 N | Aynen kalıyor: (a) 2 kıç yatay + 2 dikey | Yüzeyden dalış + w = 0.3 m/s için 14.6 N gerekiyor (kapasite 20 N) |
| 3 | Kalibrasyon bantları | **Surge 0.5–1.5 m/s** (±%6.65); heave 0.1–0.4 m/s aynı (±%11.1) | Bant altında hata büyüyor: 0.4 m/s'de +%19, 0.3 m/s'de +%40 |
| 4 | VBS | Faz 6'ya kadar **kapalı** | 0.6 L ile artık nötre inilemiyor (aşağıda) |

Kararların yan etkileri (Faz 3–6'da dikkate alınacak):
- **VBS boyutu:** %5 rezervle 0.6 L VBS net yüzdürmeyi en fazla +2.77 N'a indirebiliyor, nötre getiremiyor. Nötr için ≥ 0.88 L gerekiyor. Faz 6'da yeniden boyutlandırılacak; parametreye şimdi dokunulmadı.
- **Derinlikte tutma:** Su altında sabit 8.65 N aşağı itki gerekiyor. Bu, dikey kapasitenin %43'ü. Tutarken kalan pitch momenti ≈ 3.2 N·m (eskiden 5.9).
- **Dar bant:** Faz 3'ün kararlı itki seviyeleri ve Faz 5'in seyir hızları ≥ 0.5 m/s seçilmeli. Durgun halden ivmelenmede (Faz 4) araç bandın altından geçiyor; oradaki mutlak kuvvetler küçük (0.3 m/s'de 0.16 N).

## 1. Araç

![yerleşim](../figures/faz1/01_govde_yerlesim.png)

| | Değer | Gerekçe |
|---|---|---|
| Gövde | L = 1.20 m, D = 0.15 m (L/D = 8): kıç yarı-elipsoid 0.30 m + silindir 0.75 m + burun yarı-elipsoid 0.15 m | Tipik küçük AUV oranı; uzun kıç basınç toparlanması için. Tek parça kapalı mesh, çünkü compound hatalı (Faz 0.5). V, S, A_ön analitik olarak doğrulanabilir |
| Hacim / ıslak alan / ön kesit | 18.52 L / 0.527 m² / 176.4 cm² (mesh-tam; analitikten −%0.19 / −%0.06 / −%0.16) | 64 çevresel dilim |
| Kütle | **17.64 kg**; net yüzdürme +%5 (B − W = +8.65 N) | Yüzeyde pozitif; güç kesilince yüzer. Fribord 15.6 mm |
| Kütle dağılımı | Homojen gövde 12.64 kg + iç balast 5 kg (x_CB, z = +5.29 cm) → **BG = 1.50 cm** | Pasif roll/pitch stabilitesi; balast gövde içinde (< 0.8r) |
| Atalet (CG) | Ixx = 0.0435, Iyy = 1.226, Izz = 1.216 kg·m² (köşegen) | Gövde mesh'inin tetrahedron integrali + balast, paralel eksen |
| Akışkan | ρ = 1000 kg/m³ (tatlı su), ν = 1.138e-6 m²/s (15 °C) | Havuz testi; Stonefish varsayılan sıcaklığı |
| Thrusterlar | 4 × `simple_thruster`, ±10 N | Aşağıda |
| VBS | Tanımlı, **kapalı**: max 0.6 L → net yüzdürme +8.65 … +2.77 N | Faz 6'da yeniden boyutlandırılacak (≥ 0.88 L) |

## 2. Yüzey dengesi ve stabilite

| | Yüzey | Su altı |
|---|---|---|
| Su çekimi / fribord | 13.44 cm / **15.6 mm** (hacmin %95.2'si su altında) | — |
| Statik trim | −0.17° (burun hafif aşağı) | 0 |
| GM_T (roll) | **1.50 cm**. Formül: KB + BM_T − KG = 1.502; GZ eğimi: 1.499 | **1.575 cm** (= BG·B/W) |
| GM_L (pitch) | **40.6 cm** (formül 40.56 / sayısal 40.42); yalnızca küçük açılarda geçerli | 1.575 cm |
| Roll / pitch periyodu, su altı | — | Ek atalet yok: 0.79 / 4.21 s. Stonefish (I + aI): 0.79 / 4.73 s |

![GZ](../figures/faz1/01_gz_egrileri.png)

Yorum:
- **Roll:** Kesitler dairesel olduğu için metasantr her su çekiminde eksende. Bu yüzden yüzeyde ve su altında GM_T ≈ BG; iki eğri neredeyse çakışıyor.
- **Pitch:** Yüzeyde ince ama uzun su hattı GM_L ≈ 41 cm veriyor (%2 rezervde 26 cm'di; fribord artınca su hattı genişledi). Fribord yalnızca 15.6 mm olduğu için birkaç derecede uçlar su altına giriyor veya çıkıyor ve GZ doğrusallığını kaybediyor. Su altında pitch sertliği yaklaşık 26 kat düşüyor (GM = BG). Yüzey ↔ su altı geçişinin dinamiğini en çok bu değişim etkileyecek (Faz 6).

## 3. İtki yerleşimi ve allocation

| Thruster | Konum [m] | Yön | Rol |
|---|---|---|---|
| ThrusterPort / ThrusterStarboard | (−0.55, ∓0.10, 0) | +x | Surge + diferansiyel yaw |
| ThrusterHeaveBow / ThrusterHeaveStern | (x_CB ± 0.28, 0, 0) = (+0.303 / −0.257, 0, 0) | +z (aşağı) | Heave + diferansiyel pitch |

τ = B·T, CG etrafında (satırlar X, Y, Z, K, M, N), rank = 4:

| | Port | Stbd | Bow | Stern | Kapasite |
|---|---|---|---|---|---|
| X | 1 | 1 | 0 | 0 | 20 N |
| Z | 0 | 0 | 1 | 1 | 20 N |
| M | −0.015 | −0.015 | −0.28 | +0.28 | 5.9 N·m |
| N | +0.10 | −0.10 | 0 | 0 | 2.0 N·m |

- **Kontrol edilebilen eksenler:** X, Z, M, N.
- **Pasif kalanlar:** Y (sway; torpido gövdesinde yanal thruster yok) ve K (roll, BG ile pasif kararlı). Faz 2'de Stonefish'te roll salınımının sayısal olarak büyüdüğü görüldü; bkz. [`02_statik.md`](02_statik.md).
- Yatay thrusterlar eksen hizasında, CG ise 1.5 cm aşağıda. Bu yüzden her 1 N surge itkisi −0.015 N·m pitch (burun aşağı) momenti üretiyor. Dikey thruster farkıyla dengelenebilir (Faz 5 allocation).
- **±10 N gerekçesi (heave):** Yüzeyden dalmak için B − W = 8.65 N, w = 0.3 m/s'deki çapraz akış direnci için ≈ 5.9 N; toplam ≈ 14.6 N, kapasite 20 N.
- **Surge'de itki fazlası:** Tam surge itkisi (20 N) çıplak gövdeye fiziksel olarak 4.5 m/s, kalibre Stonefish'te 3.2 m/s verir; ikisi de çalışma bandının dışında. Hız kontrolcüde ≤ 1.5 m/s ile sınırlanacak.
- **Faz 3'e not:** Bant içi kararlı hızlar (0.5–1.5 m/s) için toplam itki yalnızca ~0.4–2.9 N. "Maksimumun %20/40/60/80'i" seviyeleri (4–16 N) bandın dışına çıkar. Faz 3 seviyeleri N cinsinden, banda göre seçilmeli.
- Yüzey dengesinde dört thruster da 5.9 cm derinlikte, yani suyun içinde. Stonefish itkiyi yalnızca thruster sudan çıkınca keser.

### Alternatif yerleşimler

| Seçenek | Artı | Eksi |
|---|---|---|
| **(a) Seçilen: 2 kıç yatay + 2 dikey tünel** | Düşük hızda ve hover'da heave/pitch kontrolü; yüzeyden kontrollü dalış; allocation basit ve ayrık | 4 aktüatör; tünel thrusterların gerçekte direnç ve verim kaybı Stonefish'te modellenmiyor |
| (b) Tek kıç thruster + rudder/elevator | Enerji verimli, gerçek torpidolara benzer; Stonefish `rudder` lift'i modelliyor | Düşük hızda kontrol yok, hover yok; yüzeyden dalış ancak ileri hızla mümkün |
| (c) Tek dikey thruster (x_CB'de) | Daha basit, 3 aktüatör | Pitch kontrolü yok (pasif BG = 1.5 cm, T ≈ 4.7 s); yüzeyde trim düzeltilemiyor |
| (d) Sadece VBS ile dalış | Sessiz, hover'da enerji harcamıyor | Yavaş (debiye bağlı); Stonefish'te VBS sadece ağırlık ekliyor, ataleti değiştirmiyor; pitch kontrolü yok |

## 4. Direnç ve Stonefish kalibrasyonu (kullanıcı kararı: toplam direnç, bant fiti)

- **Fiziksel referans:** R = ½ρ·S·C_F(Re)·(1+k)·u². C_F = ITTC-1957 = 0.075/(log Re − 2)². Form faktörü (Hoerner) 1+k = 1 + 1.5(D/L)^1.5 + 7(D/L)³ = **1.080**.
  - Sürtünme = ½ρSC_Fu², basınç (viskoz basınç) = k × sürtünme. Sürtünmenin toplamdaki payı her hızda %92.6.
- **Stonefish formu** (Faz 0.5): F = ½ρ·C_d·A_ön·u³ + ρ·C_f·S_t·u. İki katsayı, **minimax göreli hata** ile 0.5–1.5 m/s bandına fit edildi:

| | C_d (x, y, z) | C_f (x, y, z) [m/s] | Bant | Max göreli hata |
|---|---|---|---|---|
| **Kalibre (seçilen)** | 0.06323, 1.778, 1.778 | 0.001329, 0.02175, 0.02175 | surge 0.5–1.5, heave 0.1–0.4 m/s | %6.65 / %11.1 |
| Kalibre, geniş bant (Faz 1 önerisi, bırakıldı) | 0.07703, 1.778, 1.778 | 0.001036, 0.02175, 0.02175 | surge 0.3–1.5 | %13.4 |
| Stonefish varsayılan (MVAE portu; Faz 2'de canlı doğrulandı) | 0.125, 1, 1 | 0.0125, 0.1, 0.1 | — | 1 m/s'de fiziğin 5.7 katı |

| u [m/s] | Fiziksel R [N] (sürt./bas.) | Stonefish kalibre [N] (F_f/F_p) | Hata |
|---|---|---|---|
| 0.3 (bant dışı) | 0.157 (0.145 / 0.012) | 0.220 (0.205 / 0.015) | +%40.1 |
| 0.4 (bant dışı) | 0.260 (0.241 / 0.019) | 0.309 (0.273 / 0.036) | +%18.7 |
| 0.5 | 0.385 (0.357 / 0.029) | 0.411 (0.341 / 0.070) | +%6.6 |
| 1.0 | 1.320 (1.222 / 0.098) | 1.241 (0.683 / 0.558) | −%6.0 |
| 1.5 | 2.726 (2.524 / 0.202) | 2.907 (1.024 / 1.883) | +%6.6 |
| 2.0 (bant dışı) | 4.569 | 5.828 | +%27.5 |

![direnç](../figures/faz1/01_direnc_kalibrasyon.png)

- **Bant genişliği ve hata:** 0.3–1.5 m/s gibi 5 katlık bir bantta bu form fiziksel eğriyi en iyi ±%13 ile yakalayabiliyordu. 0.5–1.5 m/s'de hata ±%6.65.
- **Ayrım fiziksel değil.** Stonefish'te sürtünmenin payı bant boyunca %83'ten %35'e düşüyor (fizikte sabit %93). Faz 4'te bu açıkça gösterilecek: sim ile ITTC karşılaştırması toplamda anlamlı olacak, bileşen bazında olmayacak.
- **Heave:** Çapraz akış C_D,c = 0.8, A_plan = 0.165 m². Bu bir literatür varsayımı, **doğrulanmadı**. Max hata = %11.1 = 1/9; kuadratik bir hedefin 4 katlık banttaki teorik minimum hatası.

## 5. Ek kütle (Faz 2'de ölçüldü)

| | Surge (x) | Heave/sway (z, y) |
|---|---|---|
| Fiziksel (prolate sferoid L/D = 8, Lamb) | m_a = 0.54 kg (k₁ = 0.029) → **18.18 kg** | m_a = 17.50 kg (k₂ = 0.945) → **35.13 kg** |
| Stonefish (tek skaler: m + ort(m_a); MVAE yarı-eksenleri 0.6, 0.075, 0.075 m) | **27.22 kg** (+%50) | **27.22 kg** (−%23) |

Stonefish'te surge yaklaşık 1.5 kat "ağır", heave yaklaşık 0.77 kat "hafif" davranacak. Bu doğrudan ivmelenme zaman sabitlerine yansır ve Faz 4'teki iki modelin ayrıştığı ikinci büyük nokta.

Stonefish bu mesh için MVAE iterasyonuna hiç girmiyor (k = 0). Yükleyicinin ürettiği kopya köşeler başlangıç ağırlıklarını şişirdiği için ilk ε negatif çıkıyor; sonuçta sınırlayıcı kutunun yarı-boyutları kullanılıyor (`hybrid_vehicle_sim/mvae.py`). Faz 1'deki "(L/2, r, r)" tahmini bu yüzden birebir doğruydu.

## 6. Doğrulanan / varsayım / ölçülen

| Konu | Durum |
|---|---|
| Mesh kapalı, tutarlı yönlü (Euler = 2); dilimleme ile tetrahedron hacmi farkı 6e-7 | ✅ doğrulandı (Faz 1) |
| GM_T = BG (analitik beklenti), GM formül = sayısal (T: %0.2, L: %0.3), balast içeride, atalet köşegen, rank 4 | ✅ doğrulandı (Faz 1) |
| Stonefish'in bu mesh için V/S/kaldırma/heave direncini birebir kullanması | ✅ Faz 2 (S1, S3) |
| ITTC-57, Hoerner form faktörü, C_D,c = 0.8, Lamb ek kütle | 📚 literatür varsayımı |
| Stonefish MVAE yarı-eksenleri, varsayılan C_d, ek kütle, ek atalet | ✅ Faz 2 (MVAE portu + S2, S3, S5) |
| Yüzeyde su çekimi ve trim | ✅ Faz 2 (S4, %5 rezervle yeniden ölçüldü). %2 tasarımı için Faz 1'deki ölçüm: eksen derinliği 6.610 cm (tahmin 6.619). Not: DebugPhysics `cob` alanı köprüde yanlış dönüşümle yayınlanıyor (`ROS2SimulationManager.cpp:768`); fizik etkilenmiyor |

## 7. Riskler

- Thruster kolları ve gövdeleri ile anten/sensör çıkıntıları Stonefish'te hidrodinamik olarak **yok**. Direnç yalnızca çıplak gövdeye ait; gerçek araçta çıkıntılar toplamı tipik olarak 1.5–3 kat artırır.
- **Fribord 15.6 mm.** %2'deki 8.8 mm'den iyi, ama yine de küçük: dalga ya da birkaç derecelik trim uçları batırabilir. Yüzey modunda anten veya kule yok.
- `simple_thruster`'da reaksiyon torku, verim kaybı ve ventilasyon yok; itki thruster sudan çıktığı anda sıfırlanıyor. Ayrıca setpoint başlatılmamış (Faz 2 bulgusu): ilk komut ya da 1 s'lik watchdog gelene kadar bellekteki rastgele değer uygulanıyor.
- Kalibrasyon bant dışında hızla kötüleşiyor (0.3 m/s'de +%40, 2 m/s'de +%28).
- VBS mevcut boyutuyla nötre inemiyor (Bölüm 0).
