# Faz 0: Keşif ve köprü doğrulama

Tarih: 2026-10-04. Bu belgedeki her iddia ya bir kaynak dosyaya (`dosya:satır`) ya da canlı ölçüme dayanıyor. İkisine de dayanmayanlar **"doğrulanmadı"** olarak işaretli.

Kısaltmalar: `SF` = `~/stonefish/Library/src`, `SR` = `~/stonefish_ws/src/stonefish_ros2/src/stonefish_ros2`.

## 0. Özet: modeli doğrudan etkileyen bulgular

1. **Form direnci kuadratik değil, kübik.** `F_q = ½ρ·C_d,x·A_ön·|u|³`. Kodda yüz normal hızı normalize edilmemiş (`SF/entities/SolidEntity.cpp:1772-1779`). Probe'da iki hızda `F_q/u³` = 15.709, beklenen ½ρC_dA = 15.708. `F_q/u²` ise 4.9 ve 11.1, yani sabit değil.
2. **"Skin friction" u'da lineer.** `F_f = ρ·C_f,x·S_yan·u`, C_f [m/s] boyutunda (`SolidEntity.cpp:1784-1790, 1278-1281`). Viskozite hiçbir yerde kullanılmıyor, Reynolds bağımlılığı yok (`MaterialManager.cpp:89` sadece saklıyor). Varsayılan C_f=0.1 ile probe silindirinde Stonefish sürtünmesi ITTC-1957'nin **54–102 katı** çıkıyor (§5.3).
3. **Ek kütle eksen başına değil.** Bullet tek skaler kütle kullanıyor: `M = m + (m_a,x + m_a,y + m_a,z)/3`, tüm eksenlere aynı (`SolidEntity.cpp:657-663`, `FeatherstoneEntity.cpp:37`). Probe'da ölçülen etkin atalet 63.40 kg, tahmin 63.03 kg (%0.6). XML'den ek kütle override **yok**.
4. **Kaldırma, basınç ve kuvvet dengesi birebir tutuyor.** Kaldırma −308.190 N = ρgV, basınç 49050 Pa = ρg·5 m. Kararlı halde T + F_q + F_f = 0.0000 N.
5. **Compound gövde hacmi ve direnci bozuk hesaplıyor.** Örtüşen hacim iki kez sayılıyor ve gizli iç yüzler direnç alıyor (`SF/entities/solids/Compound.cpp:190-194, 324-350`). "Silindir + küre burun" compound'u kullanılmamalı.
6. **ROS damgaları türev almak için kullanılamaz.** Damgalar duvar saatiyle publish anında atılıyor ve burst halinde geliyor (min dt 0.17 ms). Damga zamanıyla etkin atalet 4.9 kg gibi saçma bir değer çıkıyor. Mesaj sırası × Δt=0.01 s kullanılınca doğru değer (63.4 kg) çıkıyor.
7. **`<ros_debug physics>` Stonefish'in kendi F_basınç ve F_sürtünme ayrımını veriyor** (100 Hz, her sim adımında). Faz 3-4 için birincil veri kaynağı olacak.

## 1. Sürümler ve ortam

| Bileşen | Değer | Kaynak |
|---|---|---|
| Stonefish | `b21eb8e` (`v1.5-15-gb21eb8e`, 2026-07-14), proje sürümü **1.6.0**, `/usr/local` kurulu | `git describe`, `~/stonefish/CMakeLists.txt:2`, `/usr/local/include/Stonefish/version.h` |
| stonefish_ros2 | `6646e7a` (`v1.3-31-g6646e7a`, 2025-12-04), Stonefish 1.6.0 tam sürüm ister | `stonefish_ros2/CMakeLists.txt:24` |
| Yerel yama | `ROS2Interface.cpp:697`: event camera için `rclcpp::Time(...)` cast (derleme düzeltmesi, commit'siz). Bizim kullanımımızı etkilemiyor. | `git diff` |
| Bullet | 3.26 (sim log'u) | çalışma çıktısı |
| OS / GPU | Ubuntu 22.04.5, NVIDIA 595.91.07, RTX 3050 Laptop, prime `on-demand`. Offload ile `glxinfo` → NVIDIA, sim log'u "OpenGL 4.3 contexts created" | canlı |
| ROS2 | Humble, `rosbag2_py` mevcut | canlı |
| Python | numpy 1.21.5, scipy 1.8.0, matplotlib 3.5.1, PyYAML. pandas **yok** (gerekmiyor) | canlı |
| Not | Önceki bir oturumdan kalan `demo_nodes_cpp talker` (pid 17335) çalışıyor; `/chatter` topic'i ondan geliyor, dokunulmadı. | `ros2 node list` |

## 2. Köprü doğrulama (canlı)

### 2.1 Hazır senaryo: `~/stonefish_ws/scenarios/simple_ros.scn`
```bash
source /opt/ros/humble/setup.bash && source ~/stonefish_ws/install/setup.bash
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia \
ros2 launch stonefish_ros2 stonefish_simulator.launch.py \
  simulation_data:=$HOME/stonefish/Tests/Data \
  scenario_desc:=$HOME/stonefish_ws/scenarios/simple_ros.scn \
  simulation_rate:=100.0 window_res_x:=1200 window_res_y:=800 rendering_quality:=low
```
Sonuç: Node `/stonefish_ros2/stonefish_simulator` ayakta. `/robot/imu` (`sensor_msgs/msg/Imu`) **10.00 Hz** yayınlıyor (std 3.5 ms). `/robot/desired_joint_state` 1 subscriber'a sahip. `/tf` yayında. Kapatma temiz ("process has finished cleanly").

### 2.2 Probe senaryosu: `docs/probe/probe_phase0.scn`
Proje modeli değil, sadece etiket/topic doğrulaması için. Silindir r=0.1 m, L=1.0 m, ekseni gövde x'inde. Yoğunluk 1000 kg/m³ (nötr), konum 5 m derinlik. Kıçta tek `simple_thruster`. Odometri 50 Hz, IMU 20 Hz, basınç 10 Hz. `ros_debug` açık.

| Topic | Tip | Ölçülen hız |
|---|---|---|
| `/probe/thrusters` (sub) | `std_msgs/msg/Float64MultiArray` | — |
| `/probe/thruster_state` | `stonefish_ros2/msg/ThrusterState` | 100 Hz |
| `/probe/debug/physics` | `stonefish_ros2/msg/DebugPhysics` | 100.0 Hz (her sim adımı) |
| `/probe/odometry` | `nav_msgs/msg/Odometry` | 50.0 Hz |
| `/probe/imu` | `sensor_msgs/msg/Imu` | 20 Hz |
| `/probe/pressure` | `sensor_msgs/msg/FluidPressure` | 10 Hz |

İtki adımları 20 N (15 s), 50 N (15 s), 0 N. Kayıt `docs/probe/bags/` altında (git'e girmiyor), analiz `docs/probe/analyze_probe.py` ile yapıldı. GUI ve `nogpu` modu **aynı sonucu** verdi:

| Kontrol | Sim | Elle hesap (§5 formülleri) | Fark |
|---|---|---|---|
| Kaldırma F_b,z | −308.190 N | ρgV = 308.190 N | 0 |
| Basınç @5 m | 49050.0 Pa | ρg·5 = 49050.0 Pa | 0 |
| C_d (gövde x,y,z) | 1.0, 0.5, 0.5 | silindir varsayılanı | — |
| C_f (gövde x,y,z) | 0.1, 0.05, 0.05 | 0.1·C_d | — |
| T=20 N: u_∞ | 0.3103 m/s | 0.3103 (kübik+lineer kök) | <1e-4 |
| T=20 N: F_q / F_f | −0.470 / −19.530 N | −0.469 / −19.530 | ~0 |
| T=50 N: u_∞ | 0.7065 m/s | 0.7065 | <1e-4 |
| T=50 N: F_q / F_f | −5.539 / −44.461 N | −5.539 / −44.461 | 0 |
| Kararlı halde T+F_q+F_f | 0.0000 N | 0 | — |
| Etkin atalet (nominal Δt) | 63.40 kg | m + ort.(m_a) = 63.03 kg | %0.6 |
| Etkin atalet (header damgası) | 4.9 kg | — | **damga kullanılamaz** |

Damga jitter'ı: debug dt ort 10.00 ms, std 1.15 ms, min 0.17, max 13.1 ms. Odometri dt ort 20.00 ms, std 1.6 ms, min 9.8, max 23.3 ms. Mesaj sayısı/süre = 100.0 Hz, yani sim gerçek zamanlı ve kayıp yok.

## 3. Sahne ve gövde tanımı (XML)

- **Ocean:** `<ocean><water density jerlov temperature/><waves height/><particles enabled/><current .../></ocean>` (`SF/core/ScenarioParser.cpp:643-692`). Sıvı her zaman `"Water"` adıyla, μ=1.308e-3 sabit oluşturuluyor (`:667`). Ocean tanımlı değilse SUBMERGED/FLOATING modlar SURFACE'a düşüyor (`SolidEntity.cpp:45-46`).
- **Primitive'ler:** `box, cylinder, sphere, torus, wing` + mesh için `model` (`ScenarioParser.cpp:1972-2111`). **Elipsoid ya da yarım küre primitive'i yok.** Silindirin ekseni origin'in z'si; gövde x'ine `rpy="0 1.5708 0"` ile çevrilir (probe'da doğrulandı).
- **Silindir mesh'i:** dilim sayısı N=max(⌈2πr/0.1⌉, 32). Yarıçap alan koruyacak şekilde düzeltiliyor: `R = r·√(Δφ/sin Δφ)` (`SF/graphics/OpenGLContent.cpp:2117-2119`). A_ön tam πr² çıkıyor, yan yüzey ise %0.16 büyük.
- **Mesh (`model`):** `<physical><mesh filename scale/><thickness value/><origin/></physical>` + opsiyonel `<visual>` (`:2054-2106`). Hidrodinamik yaklaşımı AUTO = elipsoid (`Polyhedron.h:55,70`).
- **Kütle override:** `<mass value>`, `<inertia xyz>`, `<cg xyz rpy>` (`:1942-1950`, uygulama `:2113-2122`). CB geometrik merkezde kalıyor, bu yüzden metasentrik ofset `<cg>` ile verilir.
- **`physics`:** `disabled|surface|floating|submerged|aerodynamic` (`:1810-1829`). `floating` = kaldırma + direnç, ek kütle yok. `submerged` = ek kütle de var. İkisi de yüzey kesişimini hesaplıyor (`SolidEntity.cpp:1842-1846`). **Hibrit araç için `submerged`.**
- **Compound:** hacimler toplanıyor, örtüşmeler iki kez sayılıyor (`Compound.cpp:190-194`). Direnç her dış parçanın tüm mesh'i üzerinden hesaplanıyor (`:324-350`), yani başka parçanın içinde kalan yüzler de direnç alıyor. Ek kütle olarak dış parçaların ortalama m_a'ları toplanıyor (`:65-68, 187, 264-266`).
- **Robot:** `<robot name fixed self_collisions>` + `<base_link>` + `<world_transform>` zorunlu (`ScenarioParser.cpp` ParseRobot). Base link Featherstone multibody olarak kuruluyor, kütlesi `getAugmentedMass()` (`FeatherstoneEntity.cpp:37-38`).

## 4. Kaldırma (buoyancy)

- **Tam dalmış:** `F_b = −ρ·V·g` (NED'de yukarı = −z), CB'den uygulanıyor (`SolidEntity.cpp:1828-1835`). Probe'da tam tuttu.
- **Yüzeyi keserken, düz deniz:** kırpılmış yüzlerden tetrahedron toplamıyla V_sub ve CB_sub hesaplanıyor, `F_b = ρ g V_sub` (`:1689-1707`). Faz 2'de statik su çekimi testi buna dayanacak.
- **Yüzeyi keserken, dalgalı deniz:** yüz başına `−n·A·derinlik(fc)·ρg` basınç integrali (`:1650-1658, 1694-1700`).
- Tam dalmışken DebugPhysics `submerged_volume = 0` gösteriyor, çünkü INSIDE dalında `Vsub` atanmıyor (`:1828-1841`). Sadece raporlama hatası, fiziği etkilemiyor.

## 5. Direnç: Stonefish'in gerçek formülü

### 5.1 Yüz bazlı hesap (`SolidEntity.cpp:1771-1790`, yüzeyde `:1661-1683`)
Her üçgen yüz için: A alan, n dış birim normal, `v_c = v_akış − (v + ω×r)`, `v_n = v_c·n` (**[m/s], normalize değil**), `v_t = v_c − v_n·n`.
- `v_n < 0` ise: `F_q,ham += v_c·|v_c|·(−v_n)·A`, yani hızın **küpü**.
- Her yüz için: `F_f,ham += v_t·A`, yani hızda **lineer**.

### 5.2 Katsayı düzeltmesi (`SolidEntity.cpp:1263-1287`)
Origin eksenindeki birim yön `d` ile `C_eff = |d_x|C_x + |d_y|C_y + |d_z|C_z` hesaplanıyor. Ardından `F_q = ½ρ·C_q,eff·F_q,ham` ve `F_f = ρ·C_f,eff·F_f,ham` (½ yok).

**Saf surge, gövde ekseni x (probe ile doğrulandı):**
```
F_q = −½ ρ C_d,x A_ön |u|³          (kübik!)
F_f = −ρ C_f,x S_t u                (lineer; S_t = Σ A(1−n_x²), silindirde yan yüzey)
(m + m̄_a) u̇ = T + F_q + F_f       (Stonefish'in uyguladığı model)
```
Kod yorumu "0.5*rho*Cd*S*v2" diyor, ama gerçekleşen u³. Bunun hata mı yoksa kasıtlı mı olduğu belirsiz. Faz 4'te "Stonefish formülü" yukarıdaki haliyle kodlanacak.

### 5.3 Varsayılan katsayılar ve override
| Yaklaşım | C_d (yaklaşım ekseninde) | C_f | Kaynak |
|---|---|---|---|
| Küre | 1, 1, 1 | 0.1·C_d | `:754-755` |
| Silindir (eksen z_H) | 0.5, 0.5, **1.0 eksenel** | 0.1·C_d | `:841-844` |
| Elipsoid (mesh AUTO) | (1/a, 1/b, 1/c)/max, yani eksenel b/a | 0.1·C_d | `:1014-1019` |

Override: `<hydrodynamics viscous_drag="Cf_x Cf_y Cf_z" quadratic_drag="Cd_x Cd_y Cd_z"/>` (`ScenarioParser.cpp:1953-1959`, uygulama `:2123`). Override **sadece katsayıları** değiştiriyor; fonksiyon biçimi (kübik + lineer) değişmiyor.

**ITTC ile kıyas** (probe silindiri, S = 2πrL, ν = 1.14e-6):

| u [m/s] | Re | C_F (ITTC-57) | F_ITTC [N] | F_f,Stonefish [N] | Oran |
|---|---|---|---|---|---|
| 0.31 | 2.7e5 | 0.00636 | 0.19 | 19.53 | ~102× |
| 0.71 | 6.2e5 | 0.00522 | 0.82 | 44.46 | ~54× |

(ITTC-57 türbülanslı sınır tabaka için. Re<1e6'da geçiş bölgesi belirsizliği var.) Sonuç: **sim ile birebir uyum fiziği değil, implementasyonu doğrular.** Fiziksel referans ITTC/literatür modeli olacak. C_f, C_d override'ı ancak belirli bir hız aralığında ITTC'ye kalibrasyon sağlayabilir; fonksiyon biçimi farklı olduğu için tüm hız aralığında tutturamaz.

## 6. Ek kütle

- Geometrik yaklaşımdan hesaplanıyor:
  - Küre: `m = ⅔πρr³` (`:745-746`).
  - Silindir: eksenel `m1 = ρπr²` (**boyutsal hata: uzunluk çarpanı eksik**), dik `m2 = ρπr²L` (`:831-832`).
  - Elipsoid: eksenel `k·(4/3)πρ a r̄²`, dik `(4/3)πρ b² a` (`:1000-1003`).
- **Kullanım:** `M = m + (m_x+m_y+m_z)/3` her eksene (`:657-663, 1076`; robotlar için `FeatherstoneEntity.cpp:37`). Ek atalet `I + aI` (`:665-671`). Ağırlık sadece `m·g` (`FeatherstoneEntity::ApplyGravity`). Probe: M_ölçülen = 63.40 kg, tahmin 31.416 + 31.62 = 63.03 kg.
- **Fiziksel karşılaştırma:** ince gövdede m_a,x ≈ k₁ρV (k₁~0.02–0.1). Stonefish ise ≈ (k₁+2)/3·ρV ≈ 0.68ρV kullanıyor. Surge'deki ivmelenme zaman sabiti sim'de fiziksel modelden uzun olacak. Faz 4'te iki model ayrı ayrı gösterilecek.
- **Lamb k-faktörü** (`:1027-1039`): `e = 1 − b²/a` (boyutsal tutarsız; literatürde `e = √(1−b²/a²)`), α₀ paydasında e³ yerine e². Ortalama alındığı için etkisi küçük olabilir. **Sayısal etkisi doğrulanmadı (Faz 4).**
- Ek kütle yüzeyde de sabit; kısmi dalmada azalma yok.
- XML'den ek kütle override **yok**.

## 7. Dalga ve yüzey etkileri

- `<waves height="h">`: h aslında ocean state, 2.0'a kırpılıyor (`SF/entities/forcefields/Ocean.cpp:45`). Dalga yüksekliği GPU'da (`OpenGLRealOcean`) üretiliyor (`:149-158, 272-273`), bu yüzden **nogpu modunda dalga yok**. Doküman etkileşimin "geliştirme aşamasında" olduğunu söylüyor (`~/stonefish/docs/environment.rst:35`).
- Dalgalı denizde sadece anlık yüzeye göre hidrostatik basınç ve ıslak yüzlerdeki direnç hesaplanıyor (Froude–Krylov benzeri, statik).
- **Modellenmeyenler:** dalga yapma direnci (R_dalga), radyasyon/difraksiyon, serbest yüzeyin ek kütleye ve dirence etkisi, gövde lift'i (lift sadece `rudder`'da var), pervane havalanması/ventilasyon.
- Thruster orijini sudan çıkınca itki tamamen sıfırlanıyor (`SF/actuators/SimpleThruster.cpp:120`; `Thruster.cpp` aynı mantık). Yüzeyde kıç thruster'ının derinliği önemli.

## 8. Aktüatörler

| Tip | XML | ROS2 arayüzü | Not |
|---|---|---|---|
| `simple_thruster` | `<propeller right><mesh/><material/><look/></propeller>`, `<specs inverted lower_thrust_limit upper_thrust_limit/>` (`ScenarioParser.cpp:3045-3105`) | Robot sub `thrusters` (Float64MultiArray), aktüatör sub `topic` (Float64) (`ROS2ScenarioParser.cpp:525-548`) | Setpoint doğrudan **N**, dinamik yok. **Faz 3-4 için önerilen.** |
| `thruster` | `<specs max_setpoint inverted_setpoint normalized_setpoint/>`, `<rotor_dynamics type=zero_order\|first_order\|yoerger\|bessa\|mechanical_pi>`, `<thrust_model type=quadratic\|deadband\|linear_interpolation\|fluid_dynamics>` (`:2735-3044`) | aynı | Gerçekçi pervane; T rotor dinamiğinden geliyor |
| `vbs` | `<volume initial><mesh filename/> ×≥2</volume>` (`:3224-3255`) | sub `topic` = debi [m³/s], pub `topic` = sıvı hacmi [m³] (`ROS2ScenarioParser.cpp:617-633`, `ROS2SimulationManager.cpp:1116-1119`) | Sadece `V·ρ·g` ağırlığı ekliyor, **atalet değişmiyor**. V=Vmin'de bile Vmin·ρ·g var, bu yüzden "boş" mesh çok küçük olmalı (`SF/actuators/VariableBuoyancy.cpp:122-140`) |
| `rudder` | `:3159` | robot sub `rudders` | Hidrofoil, lift var |
| `servo` | eklem tabanlı | sub `servos` (JointState) | Kanatçık için eklem gerekiyor |

- Robot seviyesinde `<ros_subscriber thrusters="..."/>` setpoint dizisini **aktüatör tanım sırasıyla** eşliyor. Eleman sayısı uymazsa mesaj reddediliyor (`SR/ROS2SimulationManager.cpp:1229-1239`). `<ros_publisher thrusters="..."/>` ThrusterState yayınlıyor; simple_thruster için rpm=0 (`:482-519`).
- İtki aktüatör frame'inin +x'inde uygulanıyor. Moment `(r_thr − r_CG) × F` (`SimpleThruster.cpp:126-130`).

## 9. Sensörler ve debug

| Sensör | ROS mesajı | Not / kaynak |
|---|---|---|
| `odometry` | `nav_msgs/Odometry` | frame `world_ned`, pozisyon NED, **twist sensör/gövde ekseninde** (`SF/sensors/scalar/Odometry.cpp:53`, `SR/ROS2Interface.cpp:269-290`) |
| `imu` | `sensor_msgs/Imu` | `ROS2ScenarioParser.cpp:774` |
| `pressure` | `sensor_msgs/FluidPressure` | gösterge basıncı, ρ g d (`Pressure.cpp`, `Ocean.cpp` GetPressure). Probe'da tuttu |
| `dvl` | `stonefish_ros2/DVL` | + opsiyonel altitude `Range`. **Canlı test edilmedi** |
| `gps` | `sensor_msgs/NavSatFix` | **Canlı test edilmedi** |
| `<ros_debug physics>` | `stonefish_ros2/DebugPhysics` | mass, volume, C_d, C_f, velocity (gövde), **buoyancy (dünya ekseni), damping = F_q ve skin_friction = F_f (gövde ekseni)**, wetted_surface. Her sim adımında (`ROS2ScenarioParser.cpp:303-308`, `ROS2SimulationManager.cpp:736-826`) |

Sensör etiketi: `<sensor name type rate><link name/><origin/><ros_publisher topic/></sensor>`. `history`, `noise` ve `range` opsiyonel (`ScenarioParser.cpp:3415, 3625, 3653`).

## 10. Zaman tabanı

- `/clock` yayınlanmıyor. Sim duvar saatine kilitli, sabit adımla ilerliyor (`SF/core/SimulationManager.cpp:1088-1113`; `SR/ROS2SimulationManager.cpp:93-97`). Probe'da 100.0 Hz, kayıpsız.
- Header damgası publish anında `now()` ile atılıyor (`ROS2Interface.cpp:273`). Bir GUI tick'inde birden fazla adım burst halinde publish edilebiliyor (min dt 0.17 ms).
- **Faz 3 kuralı:** türev ve ivme için DebugPhysics'in **mesaj sırası × Δt_sim** kullanılacak. Mesaj sayısı/süre oranıyla kayıp kontrolü yapılacak.

## 11. Faz 1'e etkiler (öneri, onaya tabi)

1. **Geometri:** Compound kullanılmayacak. İki seçenek var:
   - (a) Tek `cylinder` primitive'i: en basit ve analitik olarak birebir; düz uçlu, eksenel C_d=1.
   - (b) Python ile üretilmiş **tek kapalı torpido OBJ**'si (silindir + elipsoid/yarım küre uçlar): V, S ve A_ön analitik olarak hesaplanabilir. Stonefish bu durumda elipsoid yaklaşımı kullanır, eksenel C_d = b/a varsayılanı.

   Öneri: Faz 2'de önce (a) ile boru hattı kurulsun, sonra (b)'ye geçilsin. İkisinde de C_d ve C_f, `<hydrodynamics>` ile gerekçeli değerlere çekilsin.
2. **Fizik modu:** `submerged` + `buoyant="true"`. CG–CB ofseti `<cg>` ile verilecek.
3. **İtki:** Faz 3-4 için `simple_thruster` (T birebir biliniyor). Waypoint için de yeterli.
4. **Yüzey:** Kıç thruster'ının derinliği, yüzeyde su hattının altında kalacak şekilde seçilmeli; yoksa itki sıfırlanır.
5. **VBS:** Sadece ağırlık değiştiriyor. Mesh dosyaları üretilmesi gerekiyor (Python ile basit silindir OBJ'leri).
6. **Analiz:** Faz 4'te üç eğri olacak: (i) sim verisi, (ii) Stonefish formülü (kübik + lineer, M = m + m̄_a), (iii) ITTC + form faktörü + fiziksel m_a,x. (i) ile (ii) arasındaki uyum = implementasyon doğrulaması. (iii) = fiziksel referans.

## 12. Doğrulanmadı / açık noktalar

- Dalga açıkken yüzeydeki aracın davranışı (GUI şart; Faz 6).
- VBS'nin canlı çalışması (Faz 6).
- Lamb k-faktörünün sayısal etkisi ve elipsoid yaklaşımının torpido mesh'inde hangi yarı-eksenleri bulduğu (Faz 1/4).
- Yüzey kesişiminde (CROSSING_SURFACE) direncin canlı davranışı. Kod aynı kübik/lineer biçimi kullanıyor, ama canlı test edilmedi (Faz 2-3).
- DVL, GPS ve `thruster` (rotor dinamikli) tipi canlı test edilmedi.
- Yüksek yükte (çok sensör/kamera) real-time faktörünün düşüp düşmediği.
- Kübik form direncinin kasıtlı mı hata mı olduğu (upstream issue'larına bakılmadı).

## 13. Yeniden üretme

```bash
source /opt/ros/humble/setup.bash && source ~/stonefish_ws/install/setup.bash
# GUI (NVIDIA offload) veya nogpu:
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia ros2 launch stonefish_ros2 stonefish_simulator.launch.py \
  simulation_data:=$HOME/stonefish/Tests/Data \
  scenario_desc:=$HOME/stonefish_ws/src/hybrid_vehicle_sim/docs/probe/probe_phase0.scn \
  simulation_rate:=100.0 window_res_x:=1200 window_res_y:=800 rendering_quality:=low
# ros2 launch stonefish_ros2 stonefish_simulator_nogpu.launch.py simulation_data:=... scenario_desc:=... simulation_rate:=100.0

# Ayrı terminalde:
cd ~/stonefish_ws/src/hybrid_vehicle_sim/docs/probe
ros2 bag record -o bags/probe /probe/debug/physics /probe/odometry /probe/thruster_state /probe/pressure /probe/imu /probe/thrusters &
timeout 15 ros2 topic pub -r 20 /probe/thrusters std_msgs/msg/Float64MultiArray "{data: [20.0]}"
timeout 15 ros2 topic pub -r 20 /probe/thrusters std_msgs/msg/Float64MultiArray "{data: [50.0]}"
timeout 6  ros2 topic pub -r 20 /probe/thrusters std_msgs/msg/Float64MultiArray "{data: [0.0]}"
kill -INT %1
python3 analyze_probe.py bags/probe
```
