# NAUTILUS: Hibrit (yüzey + su altı) araç modeli, Stonefish

Bu projede, hem su yüzeyinde hem su altında hareket edebilen bir aracın [Stonefish](https://github.com/patrykcieslak/stonefish) simülatöründe modellenmesi amaçlanıyor. Hedef, x ekseninde sürtünme ve basınç (form) direncini ayırıp

    (m + m_a)·u̇ = T − F_sürtünme − F_basınç (− F_dalga, yüzeyde)

modelini kurmak ve simülasyonla kıyaslamak.

## Durum

| Faz | İçerik | Durum |
|---|---|---|
| 0 | Keşif, köprü doğrulama, Stonefish hidrodinamiği kaynak koddan | ✅ [`docs/00_kesif.md`](docs/00_kesif.md) |
| 0.5 | Direnç üssü, birim analizi, C_d/C_f override, zaman tabanı, tek parça mesh | ✅ [`docs/00b_dogrulama.md`](docs/00b_dogrulama.md) |
| 1 | Araç tasarım parametreleri, tek parça gövde mesh'i, stabilite, itki yerleşimi, direnç kalibrasyonu | 🟡 aşamada: [`docs/01_tasarim.md`](docs/01_tasarim.md) |
| 2 | YAML → senaryo üretimi, statik testler | ⏳ |
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

## Klasörler

```
config/              vehicle.yaml (girdi), vehicle_derived.yaml (otomatik)
hybrid_vehicle_sim/  model kütüphanesi: hull, hydrostatics, mass, drag, thrusters
scripts/             design_vehicle.py (YAML -> mesh, tablolar, grafikler)
meshes/              hull.obj (tek parça, watertight gövde)
docs/                faz raporları (00_kesif.md, 00b_dogrulama.md, 01_tasarim.md, ...)
docs/probe/          doğrulama senaryoları (.scn), test mesh'i, koşu ve analiz betikleri
figures/             rapor grafikleri (PNG + PDF)
```

Araç modeli (Faz 1):

```bash
python3 scripts/design_vehicle.py   # config/vehicle.yaml -> meshes/hull.obj, config/vehicle_derived.yaml, docs/01_tasarim_tablolar.md, figures/faz1/
```

## Simülasyonda görmek

```bash
scripts/run_sim.sh            # GUI (NVIDIA offload), 100 Hz, orta kalite; senaryoyu YAML'den yeniden üretir
scripts/run_sim.sh nogpu      # pencere yok
```

İtki komutları. Sıra: Port, Starboard, HeaveBow, HeaveStern; birim N; dikey thrusterlarda + = aşağı. 1 s watchdog var: yayın durunca itki sıfırlanır.

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
- Python: numpy, scipy, matplotlib, PyYAML, rosbag2_py

## Yeniden üretme

Probe koşuları ve analiz komutları [`docs/00b_dogrulama.md`](docs/00b_dogrulama.md) dosyasının "Yeniden üretme" bölümünde.

Not: `docs/probe/probe_phase0b.scn` mesh dosyasını mutlak yolla (`/home/burak/...`) referans ediyor. Başka bir makinede yolu güncellemek gerekiyor. Faz 2'de `$(find ...)` kullanılacak. Bag kayıtları repoya dahil değil.
