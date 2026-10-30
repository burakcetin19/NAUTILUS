# NAUTILUS: Hibrit (yüzey + su altı) araç modeli, Stonefish

Bu projede, hem su yüzeyinde hem su altında hareket edebilen bir aracın [Stonefish](https://github.com/patrykcieslak/stonefish) simülatöründe modellenmesi amaçlanıyor. Hedef, x ekseninde sürtünme ve basınç (form) direncini ayırıp

    (m + m_a)·u̇ = T − F_sürtünme − F_basınç (− F_dalga, yüzeyde)

modelini kurmak ve simülasyonla kıyaslamak.

## Durum

| Faz | İçerik | Durum |
|---|---|---|
| 0 | Keşif, köprü doğrulama, Stonefish hidrodinamiği kaynak koddan | ✅ [`docs/00_kesif.md`](docs/00_kesif.md) |
| 0.5 | Direnç üssü, birim analizi, C_d/C_f override, zaman tabanı, tek parça mesh | ✅ [`docs/00b_dogrulama.md`](docs/00b_dogrulama.md) |
| 1 | Araç tasarım parametreleri, tek parça gövde mesh'i, stabilite, itki yerleşimi, direnç kalibrasyonu | ✅ onaylandı (net yüzdürme %5, surge bandı 0.5–1.5 m/s): [`docs/01_tasarim.md`](docs/01_tasarim.md) |
| 2 | YAML → senaryo üretimi, statik testler (49/49 PASS) | 🟡 onay bekliyor: [`docs/02_statik.md`](docs/02_statik.md) |
| 3 | Açık çevrim itki testleri | ⏳ |
| 4 | Analitik x ekseni modeli ve kıyas | ⏳ |
| 5 | Waypoint takibi | ⏳ |
| 6 | Dikey hareket ve yüzey ↔ su altı geçişi | ⏳ |

## Şimdiye kadarki önemli bulgular (Stonefish `b21eb8e`)

- Form direnci hızın **küpüyle** ölçekleniyor (`½ρC_dA|u|³`); log-log fitte n = 3.000, R² = 1.0000000. Büyük olasılıkla 2024'te girmiş bir hata (`12ed0e11`).
- "Skin friction" hızla **lineer** (`ρC_fSu`), C_f [m/s] boyutunda, Reynolds bağımlılığı yok. Varsayılan C_f=0.1 ile ITTC-1957'nin 54–102 katı.
- Ek kütle eksen başına değil; üç eksenin ortalaması tüm eksenlere uygulanıyor. XML'den değiştirilemiyor.
- C_d ve C_f gövde başına `<hydrodynamics viscous_drag quadratic_drag/>` ile ayarlanabiliyor.
- Köprü `/clock` yayınlamıyor ve header damgaları duvar saati. Türev için DebugPhysics mesaj indeksi × Δt_sim kullanılıyor; ölçümlerde mesaj düşmesi görülmedi.
- Compound gövde hacmi ve direnci hatalı hesaplıyor; tek parça kapalı mesh kullanılıyor. Stonefish mesh'in hacim, yüzey, kaldırma ve direncini 1e-8 seviyesinde birebir kullanıyor.
- (Faz 2) Stonefish'in mesh için kullandığı elipsoid bu gövdede sınırlayıcı kutu (0.6, 0.075, 0.075 m): MVAE iterasyonu yükleyicinin kopya köşeleri yüzünden hiç çalışmıyor. Etkin kütle 27.219 kg = m + ort(m_a), tahminle 7.5e-6 farkla tutuyor (fiziksel: surge 18.2, heave 35.1 kg).
- (Faz 2) `simple_thruster` setpoint'i başlatılmamış: ilk komut ya da watchdog gelene kadar bellekteki rastgele değer itki olarak uygulanıyor (bir koşuda 4e89 N, sim patladı). Betikler sim'den önce sıfır itki yayınlıyor.
- (Faz 2) Kaldırma momenti 50 Hz'de tutulduğu için dalmış roll salınımı 100 Hz sim'de büyüyor (σ = −0.107 1/s); 50 Hz sim'de kararlı. Kalibre katsayıların anizotropisi ve Stonefish'in L1 katsayı karışımı yüzünden surge direnci hücum açısına çok duyarlı (1 m/s'de 0.5° → +%18).

## Klasörler

```
config/              vehicle.yaml (girdi), vehicle_derived.yaml (otomatik)
hybrid_vehicle_sim/  model kütüphanesi: hull, hydrostatics, mass, drag, thrusters,
                     mvae (Stonefish elipsoid yaklaşımı portu), simlog (bag okuma, zaman tabanı)
scripts/             design_vehicle.py (YAML -> mesh, tablolar, grafikler)
                     gen_scenario.py (derived YAML -> .scn, test varyantları), run_sim.sh (görüntüleme)
                     run_case.sh (tek test koşusu + bag), thrust_cmd.py (sabit itki yayını)
                     static_tests.sh + analyze_static.py (Faz 2)
meshes/              hull.obj (tek parça, watertight gövde), thruster.obj (görsel)
docs/                faz raporları (00_kesif.md, 00b_dogrulama.md, 01_tasarim.md, 02_statik.md, ...)
docs/probe/          Faz 0/0.5 doğrulama senaryoları (.scn), test mesh'i, koşu ve analiz betikleri
figures/             rapor grafikleri (PNG + PDF)
bags/, scenarios/    koşu kayıtları ve üretilen senaryolar (git'e girmiyor)
```

Araç modeli (Faz 1):

```bash
python3 scripts/design_vehicle.py   # config/vehicle.yaml -> meshes/, config/vehicle_derived.yaml, docs/01_tasarim_tablolar.md, figures/faz1/
```

Statik testler (Faz 2, ~10 dk, pencere yok):

```bash
scripts/static_tests.sh             # 9 koşu -> bags/faz2/
source /opt/ros/humble/setup.bash && source ~/stonefish_ws/install/setup.bash
python3 scripts/analyze_static.py   # -> docs/02_statik_tablolar.md, docs/02_statik_sonuclar.json, figures/faz2/
```

## Simülasyonda görmek

```bash
scripts/run_sim.sh            # GUI (NVIDIA offload), 100 Hz, orta kalite; senaryoyu YAML'den yeniden üretir
scripts/run_sim.sh nogpu      # pencere yok
```

İtki komutları. Sıra: Port, Starboard, HeaveBow, HeaveStern; birim N; dikey thrusterlarda + = aşağı. 1 s watchdog var: yayın durunca itki sıfırlanır.

Not: Stonefish'in `simple_thruster`'ı setpoint'i başlatmıyor. İlk komut ya da watchdog gelene kadar bellekteki rastgele değer itki olarak uygulanıyor (Faz 2'de bir koşuda 4e89 N görüldü). `run_sim.sh` bu yüzden açılışta 8 s boyunca sıfır itki yayınlıyor; o sürede gönderilen komutlar sıfırlarla karışabilir.

```bash
ros2 topic pub -r 10 /nautilus/thrusters std_msgs/msg/Float64MultiArray "{data: [1.0, 1.0, 0.0, 0.0]}"   # ileri
ros2 topic pub -r 10 /nautilus/thrusters std_msgs/msg/Float64MultiArray "{data: [1.0, -1.0, 0.0, 0.0]}"  # sağa dön
ros2 topic pub -r 10 /nautilus/thrusters std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0, 6.0, 6.0]}"   # dal
```

Kamera:
- Sağ tuşla sürükle: döndür. Orta tuşla sürükle: kaydır. Tekerlek: yakınlaştır.
- W/S/A/D/Q/Z: hareket (Shift ile hızlı). K: tuş haritası. ESC: çıkış.

## Ortam

- Ubuntu 22.04, ROS2 Humble
- Stonefish 1.6.0 (`b21eb8e`), stonefish_ros2 (`6646e7a`)
- Python: numpy, scipy, matplotlib, PyYAML, rosbag2_py, rclpy

## Yeniden üretme

Probe koşuları ve analiz komutları [`docs/00b_dogrulama.md`](docs/00b_dogrulama.md) dosyasının "Yeniden üretme" bölümünde.

Faz 2'den itibaren üretilen senaryolar mesh'lere data dizinine göre göreli yolla bakıyor; `simulation_data` paket kökü olarak veriliyor (`run_sim.sh`, `run_case.sh`). Böylece başka bir makinede yol düzenlemesi gerekmiyor. Faz 0.5 arşivindeki `docs/probe/probe_phase0b.scn` ise hâlâ mutlak yol (`/home/burak/...`) içeriyor. Bag kayıtları repoya dahil değil.
