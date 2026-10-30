# NAUTILUS: Stonefish ROS 2 Workspace

Bu repo, [Stonefish](https://github.com/patrykcieslak/stonefish) simülatörü üzerinde hibrit (yüzey + su altı) araç modelleme ve simülasyon çalışma alanını (`stonefish_ws`) içerir.

## Dizin Yapısı

- **`src/hybrid_vehicle_sim/`**: NAUTILUS hibrit araç modeli, hidrodinamik parametreler, senaryo üreteçleri, testler ve araştırma dokümanları (ayrıntılar için [`src/hybrid_vehicle_sim/README.md`](src/hybrid_vehicle_sim/README.md)).
- **`src/stonefish_ros2/`**: Stonefish simülatörünün ROS 2 köprü arayüzü.
- **`scenarios/`**: Simülasyon senaryoları.

## Derleme

```bash
colcon build --symlink-install
source install/setup.bash
```
