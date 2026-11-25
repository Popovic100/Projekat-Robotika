# Automatsko sortiranje kocki pomoću xArm7 robota u ROS 2 i Gazebo simulaciji

Projekat iz predmeta **Industrijska robotika** na Fakultetu tehničkih nauka u Novom Sadu. Sistem implementira autonomni ROS 2 kontroler za prepoznavanje, hvatanje i sortiranje kocki po bojama (crvena, zelena, plava) u 3×3 grid matricu korišćenjem kolaborativnog manipulatora **Ufactory xArm7** i **Intel RealSense D435i** RGB-D kamere.

---

## Ključne funkcionalnosti

* **Dinamičko generisanje scene:** Automatsko postavljanje nasumično raspoređenih kockica (4×4×4 cm) i statičnih ciljnih podloga (15×15 cm) u tri boje pri svakom pokretanju simulacije.
* **Računarska vizija (OpenCV + TF2):** Obrada slike sa RGB kamere u HSV prostoru boja, razlikovanje kockica od podloga na osnovu površine konture (prag od 8000 piksela) i transformacija 2D piksela u 3D koordinate baze robota (`link_base`).
* **Mašina stanja i prioriteti:** Prikupljanje pozicija u prvih 5 sekundi, zaključavanje podataka i sekvencijalno sortiranje po prioritetu boja (**crvena → zelena → plava**) u 3×3 matricu na ciljnim podlogama.
* **Planiranje putanja i sinhronizacija fizike:** Korišćenje **MoveIt 2** okruženja sa prilagođavanjem brzine (`velocity_scale = 0.4`) pri spuštanju, uz dinamičko vezivanje i otpuštanje objekata putem Gazebo `DetachableJoint` plugin-a (`/attach` i `/detach` topici).

---

## Tehnologije i okruženje

* **Operativni sistem:** Ubuntu Linux
* **Robotski okvir:** ROS 2 Jazzy
* **Simulator:** Gazebo Sim (`ros_gz_bridge`)
* **Planiranje kretanja:** MoveIt 2 / MoveItPy
* **Računarska vizija:** OpenCV, CvBridge, TF2
* **Hardver u simulaciji:** Ufactory xArm7 (7 DoF) + Intel RealSense D435i

---

## Struktura sistema (ROS 2 čvorovi)

| Čvor / Fajl | Opis funkcionalnosti |
| :--- | :--- |
| `task.launch.py` | Centralna launch skripta koja pokreće Gazebo, MoveIt, mostove (`ros_gz_bridge`), generiše kocke i ubacuje `DetachableJoint` plugin-ove. |
| `go_to_home.py` | Dovodi manipulator u početnu „Home“ konfiguraciju zglobova i objavljuje signal spremnosti na temi `/robot_ready`. |
| `vision_sorting_node.py` | Detektuje kocke i podloge pomoću HSV maski, računa 3D koordinate preko pinhole modela i objavljuje ih na `/red_position`, `/red_surface` itd. |
| `closest_cube` | Servisni čvor (`/get_closest_cube`) koji pronalazi jedinstveni `cube_id` najbliže kocke na osnovu svetskih koordinata. |
| `sorting_controller.py` | Glavni kontroler (mašina stanja) koji upravlja MoveIt putanjama, brzinom spuštanja, 3×3 grid rasporedom i Gazebo `attach`/`detach` komandama. |

---

## Uputstvo za instalaciju i pokretanje

### 1. Kompajliranje radnog prostora
U prvom terminalu inicijalizuj ROS 2 Jazzy okruženje i izgradi paket:

```bash
source /opt/ros/jazzy/setup.bash
cd E1_IR26_Projekat-master
colcon build --packages-select project_2026
source install/setup.bash
```

*(Napomena: U svakom narednom terminalu pre pokretanja komande potrebno je izvršiti `source install/setup.bash`.)*

### 2. Pokretanje simulacije i čvorova (redosled po terminalima)

**Terminal 1 – Glavna simulaciona scena:**
```bash
ros2 launch project_2026 task.launch.py
```
*(Sačekati da se Gazebo u potpunosti učita i ubace plugin-ovi za kocke.)*

**Terminal 2 – Pozicioniranje robota u Home položaj:**
```bash
ros2 run project_2026 go_to_home
```

**Terminal 3 – Statična TF2 transformacija za kameru:**
```bash
ros2 run tf2_ros static_transform_publisher --x -0.440 --y -0.008 --z 0.580 --roll -3.124 --pitch 0.004 --yaw -1.579 --frame-id link_base --child-frame-id camera_color_optical_frame
```

**Terminal 4 – Vizuelni podsistem (detekcija objekata):**
```bash
ros2 run project_2026 vision_sorting_node
```

**Terminal 5 – Servis za identifikaciju najbliže kocke:**
```bash
ros2 run project_2026 closest_cube
```

**Terminal 6 – Glavni kontroler sortiranja:**
```bash
ros2 run project_2026 sorting_controller
```

---

## Poznata ograničenja i dalji razvoj

* **Kontakti pri zahvatu:** Manji vizuelni šum pri proceni pozicije ponekad može dovesti do blagog kontakta prstiju hvataljke sa ivicom kocke ili odbijanja putanje kada su kocke preblizu jedna drugoj.
* **Sinhronizacija fizike:** Zbog kašnjenja između ROS 2 čvorova i Gazebo fizike, povremeno može doći do blagog pomeranja kocki nakon otpuštanja ili zaustavljanja na pripremnoj visini.
* **Fino podešavanje grida:** Parametri koraka (`dx`, `dy` = `0.045 m`) u 3×3 matrici mogu se dodatno kalibrisati radi još simetričnijeg slaganja na podlozi.

---

## Autori i mentori

* **Autori:** Živković Sara (EE108/2021), Popović Aleksandar (EE100/2021)
* **Mentori:** Prof. dr Babković Kalman, Prof. dr Raković Mirko
* **Institucija:** Fakultet tehničkih nauka, Univerzitet u Novom Sadu
