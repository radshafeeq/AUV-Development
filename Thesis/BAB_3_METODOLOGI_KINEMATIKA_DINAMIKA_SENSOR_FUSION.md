# BAB III: METODOLOGI PENELITIAN
## ANALISIS KINEMATIKA, DINAMIKA, DAN FUSI MULTI-SENSOR PADA AUTONOMOUS UNDERWATER VEHICLE (AUV) 6-DOF OVER-ACTUATED 8-MOTOR

---

> **Penulis**: Radhi Shafeeq  
> **NIM**: D021201000  
> **Program Studi**: Teknik Mekatronika — Departemen Teknik Mesin  
> **Fakultas**: Fakultas Teknik, Universitas Hasanuddin  
> **Pembimbing Utama**: [Nama Dosen Pembimbing]  
> **Format Penulisan**: Sesuai dengan *Pedoman Tugas Akhir Mahasiswa Fakultas Teknik Universitas Hasanuddin*  
> **Standar Notasi**: Society of Naval Architects and Marine Engineers (SNAME, 1950) & Fossen (2021)  
> **Format Matematis**: Menggunakan delimitasi `$$...$$` untuk kompabilitas konversi grafis publikasi jurnal dan dokumen skripsi.

---

## 3.1 Pendekatan dan Diagram Alir Metodologi Penelitian

Metodologi penelitian ini dirancang secara sistematis untuk memodelkan, mensimulasikan, dan menguji estimasi *state* terpadu pada wahana kapal selam otonom (*Autonomous Underwater Vehicle* / AUV) 6 *Degrees of Freedom* (6-DOF) *over-actuated* berpenggerak 8 pendorong (*thruster* Blue Robotics T200) berbasis kerangka BlueROV2 *Heavy*.

Penelitian ini memadukan empat pilar rekayasa mekatronika:
1. **Pemodelan Kinematika Matematis**: Perumusan matriks transformasi koordinat ortogonal dari kerangka gerak wahana (*Body-Fixed Frame*) ke kerangka inersial bumi (*Earth-Fixed Frame*) serta mitigasi singularitas representasi (*gimbal lock*).
2. **Pemodelan Kinetika & Hidrodinamika Non-Linear (Fossen)**: Formulasi tensor massa total (massa benda tegar dan *hydrodynamic added mass*), tensor redaman hidrodinamika coupled (gesekan laminar Navier-Stokes dan *quadratic drag*), serta gaya apung/pemulih hidrostatis.
3. **Alokasi Pendorong Over-Actuated ($$6 \times 8$$)**: Pemetaan gaya generalisasi 6-DOF ke dalam 8 sinyal modulasi lebar pulsa (*Pulse Width Modulation* / PWM) aktuator menggunakan invers semu Moore-Penrose dengan optimasi energi kuadratik minimum.
4. **Fusi Multi-Sensor & Arsitektur Dual Kalman Filter**: Penggabungan telemetri *Inertial Measurement Unit* (IMU), sensor tekanan subsea Bar30 (MS5837), umpan balik PWM motor, dan sistem visi kecerdasan buatan (kamera YOLO26) ke dalam *Extended Kalman Filter* (EKF) dinamika dan *Visual Kalman Filter* 8D.

```
       +------------------------------------------------------------------+
       |             BAB III: METODOLOGI MEKATRONIKA TERPADU              |
       +------------------------------------------------------------------+
                                        |
       +--------------------------------+---------------------------------+
       |                                                                  |
       v                                                                  v
+-------------------------------+                               +-------------------------------+
|  1. KINEMATIKA 6-DOF (SNAME)  |                               |   2. DINAMIKA FOSSEN 6-DOF    |
| • Kerangka {n} dan {b}        |                               | • Matriks Massa M = M_RB + M_A|
| • Rotasi SO(3) R_b^n          |                               | • Matriks Coriolis C(nu)      |
| • Transformasi Sudut T_Theta  |                               | • Redaman D(nu) = D_L + D_NL  |
| • Matriks Jacobian J(eta)     |                               | • Vektor Pemulih Hidrostatis g|
+-------------------------------+                               +-------------------------------+
       |                                                                  |
       +--------------------------------+---------------------------------+
                                        |
                                        v
       +------------------------------------------------------------------+
       |          3. ALOKASI KONTROL OVER-ACTUATED 8-MOTOR                |
       | • Karakteristik Gaya Dorong T200 f(PWM)                          |
       | • Matriks Konfigurasi Geometri T_{6x8}                           |
       | • Solusi Invers Semu Moore-Penrose T_{6x8}^+                     |
       +------------------------------------------------------------------+
                                        |
       +--------------------------------+---------------------------------+
       |                                                                  |
       v                                                                  v
+-------------------------------+                               +-------------------------------+
|     4. FUSI MULTI-SENSOR      |                               |   5. ARSITEKTUR KALMAN FILTER |
| • Pixhawk IMU (Gyro, Accel)   |                               | • Topside Visual EKF (YOLO26) |
| • Kompas Magnetometer LSM303D |                               | • Subsea Dynamics EKF (Pi 4B) |
| • Sensor Tekanan Bar30 MS5837 |                               | • Disturbance Observer [du,dv]|
| • Umpan Balik PWM Servos 1-8  |                               | • Mahalanobis Outlier Gating  |
+-------------------------------+                               +-------------------------------+
                                        |
                                        v
       +------------------------------------------------------------------+
       |   6. VALIDASI SISTEM: TELEMETRI HIL & SIMULASI GAZEBO HARMONIC   |
       +------------------------------------------------------------------+
```

---

## 3.2 Kerangka Referensi Koordinat SNAME (1950)

Untuk menggambarkan orientasi, posisi, dan gerak dinamika AUV di bawah air tanpa ambiguitas geometris, digunakan dua sistem koordinat tangan kanan (*right-handed orthogonal frames*) yang distandarisasi oleh *Society of Naval Architects and Marine Engineers* (SNAME, 1950) dan Fossen (2021):

```
                   Z_n (Down)
                      |
                      |    X_n (North)
                      |   /
                      |  /
                      | /
       {n} Earth-Fixed|/______________ Y_n (East)
       (Inersial)
                              
                              X_b (Surge / Bow)
                                    ^
                                   / 
                                  /
                                 /  
                                /   
                         +-----+-----+
                         |   AUV     |-----> Y_b (Sway / Starboard)
                         |  (6-DOF)  |
                         +-----+-----+
                                |
                                |
                                v
                               Z_b (Heave / Keel)
```

1. **Earth-Fixed Inertial Reference Frame $$\mathcal{F}^n = \{O_n, X_n, Y_n, Z_n\}$$ (NED)**:
   Kerangka acuan tetap bumi berorientasi *North-East-Down* (Utara-Timur-Bawah). Titik asal $$O_n$$ ditetapkan pada permukaan perairan lokal. Sumbu $$X_n$$ mengarah ke Utara geografis, $$Y_n$$ mengarah ke Timur, dan sumbu $$Z_n$$ mengarah tegak lurus ke bawah menuju pusat bumi searah medan gravitasi. Posisi linear dan sudut AUV didefinisikan terhadap kerangka $$\mathcal{F}^n$$.

2. **Body-Fixed Reference Frame $$\mathcal{F}^b = \{O_b, X_b, Y_b, Z_b\}$$**:
   Kerangka acuan bergerak yang melekat secara kaku pada struktur fisik AUV. Titik asal $$O_b$$ ditempatkan pada Pusat Gravitasi (*Center of Gravity* / CG) wahana:
   - Sumbu longitudinal $$X_b$$: Mengarah ke haluan (*forward/bow*). Gerak translasi sepanjang sumbu ini disebut **Surge** ($$u$$), dan gerak rotasi mengelilinginya disebut **Roll** ($$p$$).
   - Sumbu transversal $$Y_b$$: Mengarah ke lambung kanan (*starboard*). Gerak translasi sepanjang sumbu ini disebut **Sway** ($$v$$), dan gerak rotasi mengelilinginya disebut **Pitch** ($$q$$).
   - Sumbu normal $$Z_b$$: Mengarah ke lunas bawah (*keel/downward*). Gerak translasi sepanjang sumbu ini disebut **Heave** ($$w$$), dan gerak rotasi mengelilinginya disebut **Yaw** ($$r$$).

### Vektor Keadaan 6-DOF (Tatanama SNAME)

Tatanama matematis 6 derajat kebebasan didefinisikan melalui tiga vektor keadaan:

$$\boldsymbol{\eta} = \begin{bmatrix} \boldsymbol{\eta}_1 \\ \boldsymbol{\eta}_2 \end{bmatrix} = \begin{bmatrix} x \\ y \\ z \\ \phi \\ \theta \\ \psi \end{bmatrix} \in \mathbb{R}^6, \quad \boldsymbol{\nu} = \begin{bmatrix} \boldsymbol{\nu}_1 \\ \boldsymbol{\nu}_2 \end{bmatrix} = \begin{bmatrix} u \\ v \\ w \\ p \\ q \\ r \end{bmatrix} \in \mathbb{R}^6, \quad \boldsymbol{\tau} = \begin{bmatrix} \boldsymbol{\tau}_1 \\ \boldsymbol{\tau}_2 \end{bmatrix} = \begin{bmatrix} X \\ Y \\ Z \\ K \\ M \\ N \end{bmatrix} \in \mathbb{R}^6$$

di mana:
- $$\boldsymbol{\eta} \in \mathbb{R}^6$$: Vektor posisi dan orientasi wahana diekspresikan dalam kerangka inersial bumi $$\mathcal{F}^n$$.
  - $$\boldsymbol{\eta}_1 = [x, y, z]^T$$: Posisi translasi (meter).
  - $$\boldsymbol{\eta}_2 = [\phi, \theta, \psi]^T$$: Sudut orientasi Euler (radian): sudut geleng (*roll* $$\phi$$), sudut angguk (*pitch* $$\theta$$), dan sudut oleng (*yaw/heading* $$\psi$$).
- $$\boldsymbol{\nu} \in \mathbb{R}^6$$: Vektor kecepatan linear dan angular wahana diekspresikan dalam kerangka gerak tubuh $$\mathcal{F}^b$$.
  - $$\boldsymbol{\nu}_1 = [u, v, w]^T$$: Kecepatan linear translasi (m/s).
  - $$\boldsymbol{\nu}_2 = [p, q, r]^T$$: Kecepatan angular rotasi (rad/s).
- $$\boldsymbol{\tau} \in \mathbb{R}^6$$: Vektor gaya dan momen generalisasi yang bekerja pada tubuh wahana diekspresikan dalam $$\mathcal{F}^b$$.
  - $$\boldsymbol{\tau}_1 = [X, Y, Z]^T$$: Gaya translasi (Surge force $$X$$, Sway force $$Y$$, Heave force $$Z$$) dalam satuan Newton (N).
  - $$\boldsymbol{\tau}_2 = [K, M, N]^T$$: Momen rotasi (Roll moment $$K$$, Pitch moment $$M$$, Yaw moment $$N$$) dalam satuan Newton-meter (Nm).

---

## 3.3 Penurunan Eksak Kinematika 6-DOF

Hubungan diferensial antara laju perubahan posisi/orientasi pada kerangka inersial $$\dot{\boldsymbol{\eta}}$$ dan kecepatan tubuh wahana $$\boldsymbol{\nu}$$ dinyatakan oleh persamaan diferensial kinematika:

$$\dot{\boldsymbol{\eta}} = \mathbf{J}(\boldsymbol{\eta}_2) \boldsymbol{\nu} \iff \begin{bmatrix} \dot{\boldsymbol{\eta}}_1 \\ \dot{\boldsymbol{\eta}}_2 \end{bmatrix} = \begin{bmatrix} \mathbf{R}_b^n(\boldsymbol{\eta}_2) & \mathbf{0}_{3 \times 3} \\ \mathbf{0}_{3 \times 3} & \mathbf{T}_\Theta(\boldsymbol{\eta}_2) \end{bmatrix} \begin{bmatrix} \boldsymbol{\nu}_1 \\ \boldsymbol{\nu}_2 \end{bmatrix}$$

### 3.3.1 Matriks Rotasi Linier Ortogonal $$\mathbf{R}_b^n(\boldsymbol{\eta}_2) \in SO(3)$$

Matriks rotasi $$\mathbf{R}_b^n(\boldsymbol{\eta}_2)$$ mentransformasikan vektor kecepatan linear dari kerangka tubuh $$\mathcal{F}^b$$ ke kerangka inersial $$\mathcal{F}^n$$. Transformasi ini diturunkan melalui tiga rotasi dasar berurutan sesuai konvensi aeronautika Tait-Bryan (rotasi yaw-pitch-roll $$z-y-x$$):

1. Rotasi sudut yaw $$\psi$$ mengelilingi sumbu $$Z_n$$:
   $$\mathbf{R}_{z,\psi} = \begin{bmatrix} \cos\psi & -\sin\psi & 0 \\ \sin\psi & \cos\psi & 0 \\ 0 & 0 & 1 \end{bmatrix}$$
2. Rotasi sudut pitch $$\theta$$ mengelilingi sumbu $$Y'$$ perantara:
   $$\mathbf{R}_{y,\theta} = \begin{bmatrix} \cos\theta & 0 & \sin\theta \\ 0 & 1 & 0 \\ -\sin\theta & 0 & \cos\theta \end{bmatrix}$$
3. Rotasi sudut roll $$\phi$$ mengelilingi sumbu $$X''$$ perantara:
   $$\mathbf{R}_{x,\phi} = \begin{bmatrix} 1 & 0 & 0 \\ 0 & \cos\phi & -\sin\phi \\ 0 & \sin\phi & \cos\phi \end{bmatrix}$$

Dengan mengalikan ketiga matriks rotasi secara berurutan $$\mathbf{R}_b^n = \mathbf{R}_{z,\psi} \mathbf{R}_{y,\theta} \mathbf{R}_{x,\phi}$$, diperoleh matriks rotasi lengkap:

$$\mathbf{R}_b^n(\boldsymbol{\eta}_2) = \begin{bmatrix} 
\cos\psi\cos\theta & -\sin\psi\cos\phi + \cos\psi\sin\theta\sin\phi & \sin\psi\sin\phi + \cos\psi\sin\theta\cos\phi \\
\sin\psi\cos\theta & \cos\psi\cos\phi + \sin\psi\sin\theta\sin\phi & -\cos\psi\sin\phi + \sin\psi\sin\theta\cos\phi \\
-\sin\theta & \cos\theta\sin\phi & \cos\theta\cos\phi 
\end{bmatrix}$$

Matriks $$\mathbf{R}_b^n$$ memenuhi sifat grup Lie *Special Orthogonal Group* $$SO(3)$$:
$$\mathbf{R}_b^n (\mathbf{R}_b^n)^T = \mathbf{I}_{3 \times 3}, \quad \det(\mathbf{R}_b^n) = +1, \quad (\mathbf{R}_b^n)^{-1} = (\mathbf{R}_b^n)^T = \mathbf{R}_n^b$$

### 3.3.2 Matriks Transformasi Kecepatan Sudut Euler $$\mathbf{T}_\Theta(\boldsymbol{\eta}_2)$$

Kecepatan angular tubuh $$\boldsymbol{\nu}_2 = [p, q, r]^T$$ tidak sama secara langsung dengan turunan waktu sudut Euler $$\dot{\boldsymbol{\eta}}_2 = [\dot{\phi}, \dot{\theta}, \dot{\psi}]^T$$ karena sudut-sudut Euler diukur terhadap sumbu rotasi yang berbeda. Hubungan kinematika sudut diturunkan dengan memproyeksikan laju rotasi ke sumbu tubuh:

$$\begin{bmatrix} p \\ q \\ r \end{bmatrix} = \begin{bmatrix} \dot{\phi} \\ 0 \\ 0 \end{bmatrix} + \mathbf{R}_{x,\phi}^T \begin{bmatrix} 0 \\ \dot{\theta} \\ 0 \end{bmatrix} + \mathbf{R}_{x,\phi}^T \mathbf{R}_{y,\theta}^T \begin{bmatrix} 0 \\ 0 \\ \dot{\psi} \end{bmatrix} = \begin{bmatrix} 1 & 0 & -\sin\theta \\ 0 & \cos\phi & \cos\theta\sin\phi \\ 0 & -\sin\phi & \cos\theta\cos\phi \end{bmatrix} \begin{bmatrix} \dot{\phi} \\ \dot{\theta} \\ \dot{\psi} \end{bmatrix}$$

Dengan melakukan inversi analitis menggunakan metode matriks kofaktor-adjoin:

$$\mathbf{T}_\Theta(\boldsymbol{\eta}_2) = \begin{bmatrix} 1 & 0 & -\sin\theta \\ 0 & \cos\phi & \cos\theta\sin\phi \\ 0 & -\sin\phi & \cos\theta\cos\phi \end{bmatrix}^{-1} = \begin{bmatrix} 
1 & \sin\phi\tan\theta & \cos\phi\tan\theta \\
0 & \cos\phi & -\sin\phi \\
0 & \frac{\sin\phi}{\cos\theta} & \frac{\cos\phi}{\cos\theta}
\end{bmatrix}$$

### 3.3.3 Singularitas Representasi (*Gimbal Lock*) dan Formulasi Unit Kuaternion

Pada sudut pitch $$\theta = \pm \frac{\pi}{2} = \pm 90^\circ$$, fungsi $$\tan\theta \to \pm\infty$$ dan $$\cos\theta = 0$$, menyebabkan pembagian dengan nol (*Gimbal Lock*). Untuk wahana 6-DOF yang mampu melakukan manuver pitch vertikal penuh (`PITCH_HOLD`), sistem estimasi pada algoritma pendukung mengonversi orientasi ke dalam representasi **Unit Kuaternion**:

$$\mathbf{q} = \begin{bmatrix} \eta_q & \boldsymbol{\epsilon}_q^T \end{bmatrix}^T = \begin{bmatrix} \eta_q & \epsilon_1 & \epsilon_2 & \epsilon_3 \end{bmatrix}^T \in \mathbb{R}^4, \quad \|\mathbf{q}\| = \eta_q^2 + \epsilon_1^2 + \epsilon_2^2 + \epsilon_3^2 = 1$$

Persamaan kinematika rotasi kuaternion bersifat linier dan bebas singularitas di seluruh ruang orientasi:

$$\dot{\mathbf{q}} = \frac{1}{2} \begin{bmatrix} -\boldsymbol{\epsilon}_q^T \\ \eta_q \mathbf{I}_{3 \times 3} + \mathbf{S}(\boldsymbol{\epsilon}_q) \end{bmatrix} \boldsymbol{\nu}_2 = \frac{1}{2} \begin{bmatrix} 
-\epsilon_1 & -\epsilon_2 & -\epsilon_3 \\
\eta_q & -\epsilon_3 & \epsilon_2 \\
\epsilon_3 & \eta_q & -\epsilon_1 \\
-\epsilon_2 & \epsilon_1 & \eta_q
\end{bmatrix} \begin{bmatrix} p \\ q \\ r \end{bmatrix}$$

di mana $$\mathbf{S}(\mathbf{a})$$ merupakan operator matriks *skew-symmetric* perkalian silang (*cross-product*).

---

## 3.4 Penurunan Dinamika Non-Linear 6-DOF (Formulasi Fossen)

Berdasarkan formulasi mekanika fluida kelautan Fossen (2021), persamaan gerak dinamika 6-DOF non-linear sebuah AUV yang bergerak di dalam fluida tak berhingga dinyatakan sebagai:

$$\mathbf{M} \dot{\boldsymbol{\nu}} + \mathbf{C}(\boldsymbol{\nu}) \boldsymbol{\nu} + \mathbf{D}(\boldsymbol{\nu}) \boldsymbol{\nu} + \mathbf{g}(\boldsymbol{\eta}) = \boldsymbol{\tau}_{\text{thruster}} + \boldsymbol{\tau}_{\text{dist}}$$

### 3.4.1 Matriks Inersia Total $$\mathbf{M} = \mathbf{M}_{RB} + \mathbf{M}_A \in \mathbb{R}^{6 \times 6}$$

Inersia total merupakan penjumlahan dari inersia benda tegar (*rigid-body*) $$\mathbf{M}_{RB}$$ dan inersia massa tambahan hidrodinamika (*hydrodynamic added mass*) $$\mathbf{M}_A$$.

1. **Matriks Massa Benda Tegar $$\mathbf{M}_{RB}$$**:
   Untuk wahana berbobot massa $$m = 13.0\text{ kg}$$ dengan pusat massa pada $$O_b$$:
   $$\mathbf{M}_{RB} = \begin{bmatrix} m \mathbf{I}_{3 \times 3} & -m \mathbf{S}(\mathbf{r}_g) \\ m \mathbf{S}(\mathbf{r}_g) & \mathbf{I}_g \end{bmatrix} = \begin{bmatrix}
   m & 0 & 0 & 0 & m z_g & -m y_g \\
   0 & m & 0 & -m z_g & 0 & m x_g \\
   0 & 0 & m & m y_g & -m x_g & 0 \\
   0 & -m z_g & m y_g & I_{xx} & -I_{xy} & -I_{xz} \\
   m z_g & 0 & -m x_g & -I_{yx} & I_{yy} & -I_{yz} \\
   -m y_g & m x_g & 0 & -I_{zx} & -I_{zy} & I_{zz}
   \end{bmatrix}$$

2. **Matriks Massa Tambahan Hidrodinamika $$\mathbf{M}_A$$**:
   Ketika wahana berakselerasi di dalam air, terdapat massa fluida di sekitar lambung yang ikut terakselerasi, menimbulkan gaya reaksi berlawanan arah akselerasi. Menggunakan notasi derivatif hidrodinamika SNAME:
   $$\mathbf{M}_A = -\begin{bmatrix}
   X_{\dot{u}} & X_{\dot{v}} & X_{\dot{w}} & X_{\dot{p}} & X_{\dot{q}} & X_{\dot{r}} \\
   Y_{\dot{u}} & Y_{\dot{v}} & Y_{\dot{w}} & Y_{\dot{p}} & Y_{\dot{q}} & Y_{\dot{r}} \\
   Z_{\dot{u}} & Z_{\dot{v}} & Z_{\dot{w}} & Z_{\dot{p}} & Z_{\dot{q}} & Z_{\dot{r}} \\
   K_{\dot{u}} & K_{\dot{v}} & K_{\dot{w}} & K_{\dot{p}} & K_{\dot{q}} & K_{\dot{r}} \\
   M_{\dot{u}} & M_{\dot{v}} & M_{\dot{w}} & M_{\dot{p}} & M_{\dot{q}} & M_{\dot{r}} \\
   N_{\dot{u}} & N_{\dot{v}} & N_{\dot{w}} & N_{\dot{p}} & N_{\dot{q}} & N_{\dot{r}}
   \end{bmatrix}$$

   Karena kerangka BlueROV2 Heavy memiliki simetri bidang ganda (*bilateral symmetry* bidang port-starboard dan fore-aft) pada kecepatan operasional rendah ($$< 1.5\text{ m/s}$$), elemen non-diagonal dapat diabaikan, menghasilkan matriks inersia terkuantifikasi:

$$\mathbf{M} = \text{diag}\left[ m - X_{\dot{u}}, \, m - Y_{\dot{v}}, \, m - Z_{\dot{w}}, \, I_{xx} - K_{\dot{p}}, \, I_{yy} - M_{\dot{q}}, \, I_{zz} - N_{\dot{r}} \right]$$

$$\mathbf{M} = \text{diag}\left[ 13.0 - (-6.36), \, 13.0 - (-7.12), \, 13.0 - (-18.68), \, 0.26 - (-0.189), \, 0.23 - (-0.135), \, 0.37 - (-0.222) \right]$$

$$\mathbf{M} = \text{diag}\left[ 19.36\text{ kg}, \, 20.12\text{ kg}, \, 31.68\text{ kg}, \, 0.449\text{ kg}\cdot\text{m}^2, \, 0.365\text{ kg}\cdot\text{m}^2, \, 0.592\text{ kg}\cdot\text{m}^2 \right]$$

### 3.4.2 Matriks Gaya Coriolis dan Sentripetal $$\mathbf{C}(\boldsymbol{\nu}) = \mathbf{C}_{RB}(\boldsymbol{\nu}) + \mathbf{C}_A(\boldsymbol{\nu}_r)$$

Matriks Coriolis merepresentasikan gaya semu inersia yang muncul akibat gerak tubuh dalam koordinat berotasi. Menggunakan representasi Kirchhoff, suku ini bersifat *skew-symmetric* ($$\mathbf{C}(\boldsymbol{\nu}) = -\mathbf{C}^T(\boldsymbol{\nu})$$):

$$\mathbf{C}_{RB}(\boldsymbol{\nu}) = \begin{bmatrix} \mathbf{0}_{3 \times 3} & -m \mathbf{S}(\boldsymbol{\nu}_1) - m \mathbf{S}(\boldsymbol{\nu}_2)\mathbf{S}(\mathbf{r}_g) \\ -m \mathbf{S}(\boldsymbol{\nu}_1) + m \mathbf{S}(\mathbf{r}_g)\mathbf{S}(\boldsymbol{\nu}_2) & -\mathbf{S}(\mathbf{I}_g \boldsymbol{\nu}_2) \end{bmatrix}$$

$$\mathbf{C}_A(\boldsymbol{\nu}_r) = \begin{bmatrix}
\mathbf{0}_{3 \times 3} & -\mathbf{S}(\mathbf{A}_{11}\boldsymbol{\nu}_{r1} + \mathbf{A}_{12}\boldsymbol{\nu}_{r2}) \\
-\mathbf{S}(\mathbf{A}_{11}\boldsymbol{\nu}_{r1} + \mathbf{A}_{12}\boldsymbol{\nu}_{r2}) & -\mathbf{S}(\mathbf{A}_{21}\boldsymbol{\nu}_{r1} + \mathbf{A}_{22}\boldsymbol{\nu}_{r2})
\end{bmatrix}$$

di mana $$\mathbf{A}_{ij}$$ merupakan blok submatriks dari $$\mathbf{M}_A$$. Perbedaan antara $$X_{\dot{u}}$$ dan $$Y_{\dot{v}}$$ membangkitkan fenomena **momen Munk tak stabil** ($$(X_{\dot{u}} - Y_{\dot{v}}) u_r v_r$$) yang berusaha membelokkan wahana saat melaju miring, yang diredam secara aktif oleh umpan balik 8 pendorong.

### 3.4.3 Tensor Redaman Hidrodinamika Coupled $$\mathbf{D}(\boldsymbol{\nu})$$

Disipasi energi kinetik ke dalam air dimodelkan melalui redaman linier kulit laminar (*skin friction*) dan redaman kuadratik turbulen (*cross-flow drag*):

$$\mathbf{D}(\boldsymbol{\nu}) \boldsymbol{\nu} = \mathbf{D}_{\text{lin}} \boldsymbol{\nu} + \mathbf{D}_{\text{quad}} |\boldsymbol{\nu}| \boldsymbol{\nu}$$

$$\mathbf{D}_{\text{lin}} = \text{diag}[13.7\text{ Ns/m}, \, 0.0, \, 33.8\text{ Ns/m}, \, 0.0, \, 0.0, \, 0.0]$$

$$\mathbf{D}_{\text{quad}} = \text{diag}[141.0\text{ Ns}^2/\text{m}^2, \, 217.0\text{ Ns}^2/\text{m}^2, \, 190.0\text{ Ns}^2/\text{m}^2, \, 4.0\text{ Nms}^2/\text{rad}^2, \, 4.0\text{ Nms}^2/\text{rad}^2, \, 4.0\text{ Nms}^2/\text{rad}^2]$$

### 3.4.4 Vektor Gaya Pemulih Hidrostatis 6-DOF $$\mathbf{g}(\boldsymbol{\eta})$$

Gaya berat $$W = m g$$ bekerja pada Pusat Gravitasi ($$CG = [x_g, y_g, z_g]^T$$), sedangkan gaya apung Archimedes $$B = \rho g \nabla$$ bekerja pada Pusat Daya Apung (*Center of Buoyancy* / $$CB = [x_b, y_b, z_b]^T$$). Untuk wahana yang dirancang netral ($$W \approx B$$):

$$\mathbf{g}(\boldsymbol{\eta}) = \begin{bmatrix}
(W - B) \sin\theta \\
-(W - B) \cos\theta \sin\phi \\
-(W - B) \cos\theta \cos\phi \\
-(y_g W - y_b B)\cos\theta\cos\phi + (z_g W - z_b B)\cos\theta\sin\phi \\
(z_g W - z_b B)\sin\theta + (x_g W - x_b B)\cos\theta\cos\phi \\
-(x_g W - x_b B)\cos\theta\sin\phi - (y_g W - y_b B)\sin\theta
\end{bmatrix}$$

Untuk wahana simetris dengan separasi metasentris vertikal $$\overline{BG}_z = z_g - z_b \approx 0.02\text{ m}$$:
$$\mathbf{g}(\boldsymbol{\eta}) \approx \begin{bmatrix} 0 \\ 0 \\ 0 \\ \rho g \nabla \overline{BG}_z \cos\theta \sin\phi \\ \rho g \nabla \overline{BG}_z \sin\theta \\ 0 \end{bmatrix}$$

---

## 3.5 Alokasi Pendorong Over-Actuated 8-Motor (Aktuator T200)

Pada konfigurasi BlueROV2 Heavy, wahana memiliki 8 motor pendorong independen ($$m = 8$$) untuk mengendalikan 6 DOF ($$n = 6$$).

```
                      HALUAN (Bow / +X_b)
             Thruster 1 \      ^      / Thruster 2
                         \     |     /
                          \    |    /
       Thruster 5 [Vert]   \   |   /   Thruster 6 [Vert]
       (Port-Fore)          +--+--+    (Starboard-Fore)
                            | AUV |
                            |     |
       Thruster 7 [Vert]    +--+--+    Thruster 8 [Vert]
       (Port-Aft)           /  |  \    (Starboard-Aft)
                           /   |   \
                          /    |    \
             Thruster 3  /     |     \ Thruster 4
                     BURITAN (Stern / -X_b)
```

### 3.5.1 Geometri Matriks Konfigurasi Alokasi Pendorong $$\mathbf{T}_{6 \times 8}$$

Setiap pendorong $$i$$ ($$i = 1 \dots 8$$) ditempatkan pada posisi vektor lengan momen $$\mathbf{r}_i = [x_i, y_i, z_i]^T$$ dan orientasi vektor satuan gaya $$\mathbf{u}_i = [u_{xi}, u_{yi}, u_{zi}]^T$$. Kolom ke-$$i$$ dari matriks konfigurasi alokasi pendorong $$\mathbf{T}_{6 \times 8}$$ dinyatakan oleh:

$$\mathbf{t}_i = \begin{bmatrix} \mathbf{u}_i \\ \mathbf{r}_i \times \mathbf{u}_i \end{bmatrix} \in \mathbb{R}^6$$

Hubungan antara gaya dorong 8 motor $$\mathbf{f} = [f_1, f_2, \dots, f_8]^T \in \mathbb{R}^8$$ dan gaya generalisasi tubuh $$\boldsymbol{\tau} \in \mathbb{R}^6$$ adalah:

$$\boldsymbol{\tau} = \mathbf{T}_{6 \times 8} \mathbf{f}$$

Dengan sudut canting pendorong horizontal $$45^\circ$$ ($$c = \cos 45^\circ \approx 0.7071$$) dan parameter geometri kerangka ($$d_x = 0.156\text{ m}, d_y = 0.111\text{ m}, d_z = 0.085\text{ m}$$):

$$\mathbf{T}_{6 \times 8} = \begin{bmatrix}
0.7071 & 0.7071 & -0.7071 & -0.7071 & 0 & 0 & 0 & 0 \\
-0.7071 & 0.7071 & -0.7071 & 0.7071 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & -1.0 & -1.0 & -1.0 & -1.0 \\
0 & 0 & 0 & 0 & -d_{y5} & d_{y6} & -d_{y7} & d_{y8} \\
0 & 0 & 0 & 0 & d_{x5} & d_{x6} & -d_{x7} & -d_{x8} \\
-0.177 & 0.177 & 0.177 & -0.177 & 0 & 0 & 0 & 0
\end{bmatrix}$$

### 3.5.2 Solusi Invers Semu Moore-Penrose Kuadratik Minimum

Karena jumlah aktuator lebih besar daripada derajat kebebasan ($$8 > 6$$), sistem memiliki redundansi kontrol (*over-actuated*). Solusi alokasi yang meminimalkan konsumsi energi total aktuator ($$J = \frac{1}{2} \mathbf{f}^T \mathbf{W} \mathbf{f}$$) diselesaikan menggunakan invers semu terbobot (*weighted Moore-Penrose pseudo-inverse*):

$$\mathbf{f} = \mathbf{T}^+ \boldsymbol{\tau} = \mathbf{W}^{-1} \mathbf{T}^T (\mathbf{T} \mathbf{W}^{-1} \mathbf{T}^T)^{-1} \boldsymbol{\tau}$$

---

## 3.6 Integrasi dan Pemodelan Matematika Setiap Sensor

Semua sensor fisik dan piranti deteksi pada wahana dimodelkan matematis ke dalam ruang keadaan sistem:

```
+-----------------------------------------------------------------------------------------------+
|                                  ARSITEKTUR MULTI-SENSOR AUV                                  |
+-----------------------------------------------------------------------------------------------+
| 1. InvenSense MPU-6000 & L3GD20 (SPI 1):                                                      |
|    • Gyroscope:      z_gyro = [p, q, r]^T + b_g + w_g                                         |
|    • Accelerometer:  z_acc  = R_n^b ( \dot{\nu}_1 - g^n ) + b_a + w_a                         |
+-----------------------------------------------------------------------------------------------+
| 2. STMicroelectronics LSM303D (SPI 1):                                                        |
|    • Magnetometer:   z_mag  = R_n^b B_earth + b_m + w_m  ===>  Heading \psi                   |
+-----------------------------------------------------------------------------------------------+
| 3. Blue Robotics Bar30 / MS5837-30BA (I2C 1, Addr 0x76):                                      |
|    • Hydrostatic:    P = \rho g z + P_atm                ===>  Depth z, Heave speed w = \dot{z}|
+-----------------------------------------------------------------------------------------------+
| 4. ArduSub Motor PWM Telemetry (Channels 1 - 8):                                              |
|    • ESC Inputs:     PWM_i in [1100, 1900] \mu s         ===>  \tau = T_{6x8} f(PWM)          |
+-----------------------------------------------------------------------------------------------+
| 5. Topside Vision AI (Kamera + YOLO26):                                                       |
|    • Bounding Box:   [x_c, y_c, w_c, h_c, conf]          ===>  Bearing (e_x, e_y), Range Z_c  |
+-----------------------------------------------------------------------------------------------+
```

1. **Tri-Axial Gyroscope (MPU-6000)**:
   Mengukur kecepatan angular tubuh:
   $$\mathbf{z}_{\text{gyro}} = \boldsymbol{\nu}_2 + \mathbf{b}_g + \mathbf{w}_g = \begin{bmatrix} p \\ q \\ r \end{bmatrix} + \mathbf{b}_g + \mathbf{w}_g$$
   di mana $$\mathbf{b}_g$$ adalah bias *drift* dan $$\mathbf{w}_g \sim \mathcal{N}(\mathbf{0}, \mathbf{R}_{\text{gyro}})$$ adalah *noise* instrumen putih Gauss.

2. **Tri-Axial Accelerometer (MPU-6000 & LSM303D)**:
   Mengukur gaya spesifik (*specific force*) yang mencakup akselerasi kinematis dan gravitasi:
   $$\mathbf{z}_{\text{acc}} = \dot{\boldsymbol{\nu}}_1 + \mathbf{S}(\boldsymbol{\nu}_2)\boldsymbol{\nu}_1 - (\mathbf{R}_b^n)^T \begin{bmatrix} 0 \\ 0 \\ g \end{bmatrix} + \mathbf{b}_a + \mathbf{w}_a$$
   Saat wahana diam/kecepatan konstan, akselerometer mengukur proyeksi vektor gravitasi, memberikan estimasi absolut sudut roll $$\phi$$ dan pitch $$\theta$$ tanpa *drift*.

3. **Digital Compass / Magnetometer (LSM303D)**:
   Mengukur vektor medan magnet bumi $$\mathbf{B}^n$$ terproyeksi ke tubuh wahana:
   $$\mathbf{z}_{\text{mag}} = (\mathbf{R}_b^n)^T \mathbf{B}^n + \mathbf{b}_m + \mathbf{w}_m$$
   Setelah kompensasi distorsi *hard iron* dan *soft iron*, sudut *heading* $$\psi$$ dihitung tanpa akumulasi kesalahan integrasi gyro.

4. **Sensor Tekanan Subsea Bar30 (MS5837-30BA)**:
   Tekanan hidrostatik absolut berhubungan linier dengan kedalaman fluida $$z$$:
   $$P_{\text{abs}} = \rho g z + P_{\text{atm}}$$
   $$z_{\text{meas}} = \frac{P_{\text{abs}} - P_{\text{atm}}}{\rho g}$$
   Turunan waktu dari sinyal kedalaman terfilter memberikan estimasi kecepatan vertikal (*heave velocity*) $$w_m = \frac{dz}{dt}$$.

5. **Umpan Balik PWM Motor (ArduSub `SERVO_OUTPUT_RAW`)**:
   Sinyal PWM dari 8 saluran pendorong dikonversi ke gaya dorong Newton melalui pemodelan polinomial non-linear karakteristik baling-baling T200 ($$f_i = k_{\text{pwm}} (\text{PWM}_i - 1500) |\text{PWM}_i - 1500|$$), yang kemudian dialokasikan melalui $$\boldsymbol{\tau} = \mathbf{T}_{6 \times 8} \mathbf{f}$$.

6. **Kamera & Deteksi Target YOLO26**:
   Kamera monokular memproyeksikan target 3D ke bidang citra 2D:
   $$x_p = f_x \frac{X_c}{Z_c} + c_x, \quad y_p = f_y \frac{Y_c}{Z_c} + c_y$$
   Luas kotak pembatas $$\mathcal{A} = w \cdot h$$ memiliki laju ekspansi diferensial terhadap jarak fisik $$Z_c$$:
   $$\dot{Z}_c = -\frac{Z_c}{2} \left( \frac{\dot{w}}{w} + \frac{\dot{h}}{h} \right)$$
   memberikan pengukuran kecepatan pendekatan haluan (*surge approach velocity*) secara visual.

---

## 3.7 Formulasi Lengkap Dual Kalman Filter

Sistem estimasi mengoperasikan dua Kalman filter yang terkoordinasi:

### 3.7.1 Subsea Hydrodynamic Dynamics EKF (`AUVDynamicsKalmanFilter`)

Berjalan pada Raspberry Pi 4B pada frekuensi $$50\text{ Hz}$$. State vector diperluas dengan pengamat gangguan arus laut (*Disturbance Observer*):

$$\mathbf{x}_{\text{dyn}} = \begin{bmatrix} u & v & w & p & q & r & d_u & d_v \end{bmatrix}^T \in \mathbb{R}^8$$

1. **Tahap Prediksi**:
   $$\hat{\mathbf{x}}_{k|k-1} = \hat{\mathbf{x}}_{k-1|k-1} + \mathbf{f}_c(\hat{\mathbf{x}}_{k-1|k-1}, \boldsymbol{\tau}) \Delta t$$
   $$\mathbf{P}_{k|k-1} = \boldsymbol{\Phi}_k \mathbf{P}_{k-1|k-1} \boldsymbol{\Phi}_k^T + \mathbf{Q}_{\text{dyn}}$$

   di mana matriks transisi keadaan linier diskrit $$\boldsymbol{\Phi}_k = \mathbf{I}_8 + \mathbf{F}_c \Delta t$$, dengan matriks Jacobian analitis eksak $$\mathbf{F}_c = \frac{\partial \mathbf{f}_c}{\partial \mathbf{x}} \in \mathbb{R}^{8 \times 8}$$:

$$\mathbf{F}_c = \begin{bmatrix}
-\frac{X_u + 2 X_{u|u|} |u|}{M_u} & 0 & 0 & 0 & 0 & 0 & \frac{1}{M_u} & 0 \\
0 & -\frac{Y_v + 2 Y_{v|v|} |v|}{M_v} & 0 & 0 & 0 & 0 & 0 & \frac{1}{M_v} \\
0 & 0 & -\frac{Z_w + 2 Z_{w|w|} |w|}{M_w} & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & -\frac{K_p + 2 K_{p|p|} |p|}{M_p} & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & -\frac{M_q + 2 M_{q|q|} |q|}{M_q} & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & -\frac{N_r + 2 N_{r|r|} |r|}{M_r} & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 & 0 & 0
\end{bmatrix}$$

2. **Tahap Pembaruan Pengukuran Multi-Sensor**:
   Vektor pengukuran $$\mathbf{z}_k = [u_m, v_m, w_m, p_m, q_m, r_m]^T \in \mathbb{R}^6$$:
   $$\mathbf{y}_k = \mathbf{z}_k - \mathbf{H} \hat{\mathbf{x}}_{k|k-1}, \quad \mathbf{H} = \begin{bmatrix} \mathbf{I}_{6 \times 6} & \mathbf{0}_{6 \times 2} \end{bmatrix}$$
   $$\mathbf{S}_k = \mathbf{H} \mathbf{P}_{k|k-1} \mathbf{H}^T + \mathbf{R}_{\text{dyn}}$$
   $$\mathbf{K}_k = \mathbf{P}_{k|k-1} \mathbf{H}^T \mathbf{S}_k^{-1}$$
   $$\hat{\mathbf{x}}_{k|k} = \hat{\mathbf{x}}_{k|k-1} + \mathbf{K}_k \mathbf{y}_k$$
   $$\mathbf{P}_{k|k} = (\mathbf{I}_8 - \mathbf{K}_k \mathbf{H}) \mathbf{P}_{k|k-1}$$

### 3.7.2 Topside Visual Target Kalman Filter (`AUVVisualKalmanFilter`)

Berjalan pada workstation topside (NVIDIA GPU) memproses koordinat deteksi objek YOLO26:
$$\mathbf{x}_{\text{vis}} = \begin{bmatrix} x_c & y_c & w & h & v_x & v_y & v_w & v_h \end{bmatrix}^T \in \mathbb{R}^8$$
Menggunakan model *Continuous White Noise Acceleration* (CWNA), filter ini mengeliminasi *jitter* deteksi piksel, memvalidasi pengukuran melalui *Mahalanobis distance innovation gating* ($$d_M^2 \le \gamma$$), dan melakukan interpolasi *dead-reckoning* saat target terhalang gelembung/sedimen hingga 15 frame berturut-turut.

---

## 3.8 Rangkuman Parameter dan Spesifikasi Wahana

| Simbol Parameter | Keterangan Fisik | Nilai Kuantitatif | Satuan |
| :--- | :--- | :--- | :--- |
| $$m$$ | Massa total wahana di udara | $$13.0$$ | $$\text{kg}$$ |
| $$\nabla$$ | Volume perpindahan air | $$0.0130$$ | $$\text{m}^3$$ |
| $$\rho$$ | Massa jenis air tawar / laut | $$1000.0 \ / \ 1025.0$$ | $$\text{kg/m}^3$$ |
| $$I_{xx}, I_{yy}, I_{zz}$$ | Momen inersia benda tegar utama | $$0.26, \, 0.23, \, 0.37$$ | $$\text{kg}\cdot\text{m}^2$$ |
| $$X_{\dot{u}}, Y_{\dot{v}}, Z_{\dot{w}}$$ | *Added mass* translasi | $$-6.36, \, -7.12, \, -18.68$$ | $$\text{kg}$$ |
| $$K_{\dot{p}}, M_{\dot{q}}, N_{\dot{r}}$$ | *Added mass* momen rotasi | $$-0.189, \, -0.135, \, -0.222$$ | $$\text{kg}\cdot\text{m}^2$$ |
| $$M_u, M_v, M_w$$ | Inersia total translasi | $$19.36, \, 20.12, \, 31.68$$ | $$\text{kg}$$ |
| $$M_p, M_q, M_r$$ | Inersia total rotasi | $$0.449, \, 0.365, \, 0.592$$ | $$\text{kg}\cdot\text{m}^2$$ |
| $$\overline{BG}_z$$ | Separasi Pusat Daya Apung & Pusat Gravitasi | $$0.02$$ | $$\text{m}$$ |
| $$X_u, Z_w$$ | Koefisien redaman hidrodinamika linier | $$13.7, \, 33.8$$ | $$\text{Ns/m}$$ |
| $$X_{u|u|}, Y_{v|v|}, Z_{w|w|}$$ | Koefisien redaman kuadratik translasi | $$141.0, \, 217.0, \, 190.0$$ | $$\text{Ns}^2/\text{m}^2$$ |
| $$K_{p|p|}, M_{q|q|}, N_{r|r|}$$ | Koefisien redaman kuadratik rotasi | $$4.0, \, 4.0, \, 4.0$$ | $$\text{Nms}^2/\text{rad}^2$$ |
| $$N_{\text{thruster}}$$ | Jumlah pendorong aktuasi | $$8\times \text{ T200}$$ | - |
| $$\mathbf{T}_{6 \times 8}$$ | Matriks alokasi geometri pendorong | Matriks ukuran $$6 \times 8$$ | - |
