# BAGIAN AWAL PROPOSAL TUGAS AKHIR (FRONT MATTER)

> **Catatan Format Berdasarkan Pedoman Unhas 2023 (SK Rektor No. 10438/UN4.1/KEP/2023)*:  
> Seluruh bagian awal naskah proposal diberi nomor halaman dengan angka romawi kecil (i, ii, iii, iv, v, dst.*) yang diletakkan pada sembir kanan atas. Naskah dicetak pada kertas format B5 (176 mm x 250 mm) atau A4 (disesuaikan dengan kebutuhan seminar proposal di departemen), font utama Arial 10 pt (spasi 1,15), dan judul/subjudul Arial 11 pt ditebalkan (*bold).

---

<!-- ======================================================================= -->
<!-- HALAMAN SAMPUL DEPAN (FRONT COVER)                                      -->
<!-- ======================================================================= -->

<div align="center">

# PROPOSAL TUGAS AKHIR

<br>

### *ANALISIS KINEMATIKA, DINAMIKA, DAN ESTIMASI KEADAAN OPTIMAL *KALMAN FILTER* UNTUK *VISION-BASED TRACKING* PADA OVER-ACTUATED 8-THRUSTER 6-DOF VECTORED AUV*

<br>

(ANALYSIS OF KINEMATICS, DYNAMICS, AND OPTIMAL KALMAN FILTER STATE ESTIMATION FOR VISION-BASED TRACKING IN AN OVER-ACTUATED 8-THRUSTER 6-DOF VECTORED AUV)*

<br><br>

*(Ilustrasi / Desain Grafis Model 3D AUV 8-Pendorong)  
```text
               [ V5 (Port-Fore) ]      [ V6 (Stbd-Fore) ]
                       \                /
             [ H1 ] ----+--------------+---- [ H2 ]
                        |  ROV PIP     |
                        | (6-DOF HAUV) |
             [ H3 ] ----+--------------+---- [ H4 ]
                       /                \
               [ V7 (Port-Aft) ]       [ V8 (Stbd-Aft) ]
```

<br><br>

*MUH. RADHI SYAFIQ GHANIM. S**  
**NIM. D021201006**

<br><br>

<img src="https://upload.wikimedia.org/wikipedia/commons/2/20/Logo-unhas.png" alt="Logo Universitas Hasanuddin" width="120" height="150"/>

<br><br>

**DEPARTEMEN TEKNIK MESIN**  
**FAKULTAS TEKNIK**  
**UNIVERSITAS HASANUDDIN**  
**MAKASSAR**  
**2026*

</div>

<div style="page-break-after: always;"></div>

---

<!-- ======================================================================= -->
<!-- HALAMAN JUDUL (TITLE PAGE - HALAMAN i)                                  -->
<!-- ======================================================================= -->

<div align="center">

Halaman i (Dihitung, tidak dicetak)

# PROPOSAL TUGAS AKHIR

<br>

### *ANALISIS KINEMATIKA, DINAMIKA, DAN ESTIMASI KEADAAN OPTIMAL *KALMAN FILTER* UNTUK *VISION-BASED TRACKING* PADA OVER-ACTUATED 8-THRUSTER 6-DOF VECTORED AUV*

<br>

ANALYSIS OF KINEMATICS, DYNAMICS, AND OPTIMAL KALMAN FILTER STATE ESTIMATION FOR VISION-BASED TRACKING IN AN OVER-ACTUATED 8-THRUSTER 6-DOF VECTORED AUV

<br><br><br>

*MUH. RADHI SYAFIQ GHANIM. S**  
**NIM. D021201006**

<br><br><br>

<img src="https://upload.wikimedia.org/wikipedia/commons/2/20/Logo-unhas.png" alt="Logo Universitas Hasanuddin" width="100"/>

<br><br><br>

**DEPARTEMEN TEKNIK MESIN**  
**FAKULTAS TEKNIK**  
**UNIVERSITAS HASANUDDIN**  
**MAKASSAR**  
**2026*

</div>

<div style="page-break-after: always;"></div>

---

<!-- ======================================================================= -->
<!-- HALAMAN PENGAJUAN (SUBMISSION PAGE - HALAMAN ii)                        -->
<!-- ======================================================================= -->

<div align="center">

Halaman ii (Dihitung, tidak dicetak)

### *ANALISIS KINEMATIKA, DINAMIKA, DAN ESTIMASI KEADAAN OPTIMAL *KALMAN FILTER* UNTUK *VISION-BASED TRACKING* PADA OVER-ACTUATED 8-THRUSTER 6-DOF VECTORED AUV**

<br><br>

**MUH. RADHI SYAFIQ GHANIM. S**  
**NIM. D021201006**

<br><br><br>

**Proposal Tugas Akhir**  
sebagai salah satu syarat untuk mencapai gelar sarjana  
Program Studi Teknik Mesin

<br><br>

pada

<br><br>

**DEPARTEMEN TEKNIK MESIN**  
**FAKULTAS TEKNIK**  
**UNIVERSITAS HASANUDDIN**  
**MAKASSAR**  
**2026*

</div>

<div style="page-break-after: always;"></div>

---

<!-- ======================================================================= -->
<!-- HALAMAN PENGESAHAN (APPROVAL PAGE - HALAMAN iii)                        -->
<!-- ======================================================================= -->

<div align="center">

Halaman iii (Dihitung, tidak dicetak)

# LEMBAR PENGESAHAN PROPOSAL TUGAS AKHIR

<br>

### *ANALISIS KINEMATIKA, DINAMIKA, DAN ESTIMASI KEADAAN OPTIMAL *KALMAN FILTER* UNTUK *VISION-BASED TRACKING* PADA OVER-ACTUATED 8-THRUSTER 6-DOF VECTORED AUV**

<br>

Disusun dan diajukan oleh:

<br>

**MUH. RADHI SYAFIQ GHANIM. S**  
**NIM. D021201006*

<br><br>

Telah dipertahankan di hadapan Panitia Ujian Seminar Proposal Tugas Akhir  
pada tanggal ......................... 2026  
dan dinyatakan telah memenuhi syarat kelayakan proposal penelitian  
pada Program Studi Teknik Mesin, Departemen Teknik Mesin,  
Fakultas Teknik, Universitas Hasanuddin.

<br><br><br>

</div>

<table style="width:100%; border:none; text-align:center;">
  <tr>
    <td style="width:50%; border:none;">
      Mengesahkan:<br>
      <strong>Pembimbing Utama</strong><br><br><br><br><br>
      <u>(Nama Lengkap Pembimbing Utama & Gelar)</u><br>
      NIP. ....................................................
    </td>
    <td style="width:50%; border:none;">
      Mengesahkan:<br>
      <strong>Pembimbing Pendamping</strong><br><br><br><br><br>
      <u>(Nama Lengkap Pembimbing Pendamping & Gelar)</u><br>
      NIP. ....................................................
    </td>
  </tr>
  <tr>
    <td colspan="2" style="border:none;"><br><br></td>
  </tr>
  <tr>
    <td colspan="2" style="border:none; text-align:center;">
      Mengetahui:<br>
      <strong>Ketua Program Studi Teknik Mesin</strong><br>
      Departemen Teknik Mesin, Fakultas Teknik, Universitas Hasanuddin<br><br><br><br><br>
      <u>(Nama Lengkap Ketua Program Studi & Gelar)</u><br>
      NIP. ....................................................
    </td>
  </tr>
</table>

<div style="page-break-after: always;"></div>

---

<!-- ======================================================================= -->
<!-- LEMBAR PERNYATAAN KEASLIAN (STATEMENT OF ORIGINALITY - HALAMAN iv)     -->
<!-- ======================================================================= -->

<div align="center">

Halaman iv*

# PERNYATAAN KEASLIAN PROPOSAL TUGAS AKHIR DAN PELIMPAHAN HAK CIPTA

</div>

<br>

Dengan ini saya menyatakan bahwa proposal tugas akhir yang berjudul:

<div align="center">
<strong>“ANALISIS KINEMATIKA, DINAMIKA, DAN ESTIMASI KEADAAN OPTIMAL *KALMAN FILTER* UNTUK *VISION-BASED TRACKING* PADA *OVER-ACTUATED 8-THRUSTER 6-DOF VECTORED AUV”</strong>
</div>

adalah benar merupakan karya ilmiah orisinal saya sendiri di bawah arahan dan bimbingan tim pembimbing:
1. *Pembimbing Utama**: [Nama Lengkap dan Gelar Pembimbing Utama]
2. **Pembimbing Pendamping*: [Nama Lengkap dan Gelar Pembimbing Pendamping]

Karya ilmiah ini belum pernah diajukan dan tidak sedang diajukan dalam bentuk apa pun kepada perguruan tinggi mana pun untuk memperoleh gelar akademik. Semua sumber informasi yang berasal atau dikutip dari karya ilmiah yang diterbitkan maupun tidak diterbitkan dari penulis lain telah dirujuk dan disebutkan dengan benar dalam teks serta dicantumkan dalam Daftar Pustaka naskah ini.

Apabila di kemudian hari terbukti atau dapat dibuktikan bahwa sebagian atau keseluruhan naskah proposal tugas akhir ini merupakan hasil plagiasi, fabrikasi, atau falsifikasi karya orang lain, maka saya bersedia menerima sanksi akademik yang tegas sesuai dengan peraturan perundang-undangan yang berlaku di lingkungan Universitas Hasanuddin dan Republik Indonesia.

Dengan ini saya juga melimpahkan hak cipta (hak ekonomis) dari karya tulis ilmiah ini kepada Universitas Hasanuddin.

<br><br>

<table style="width:100%; border:none;">
  <tr>
    <td style="width:50%; border:none;"></td>
    <td style="width:50%; border:none; text-align:center;">
      Makassar, ......................... 2026<br>
      Yang membuat pernyataan,<br><br>
      <em>(Materai Rp 10.000,- & Tanda Tangan)</em><br><br><br><br>
      <strong>MUH. RADHI SYAFIQ GHANIM. S</strong><br>
      NIM. D021201006
    </td>
  </tr>
</table>

<div style="page-break-after: always;"></div>

---

<!-- ======================================================================= -->
<!-- PRAKATA / UCAPAN TERIMA KASIH (PREFACE - HALAMAN v)                    -->
<!-- ======================================================================= -->

<div align="center">

Halaman v

# PRAKATA

</div>

<br>

Puji dan syukur penulis panjatkan ke hadirat Tuhan Yang Maha Esa atas berkat, rahmat, dan karunia-Nya yang melimpah, sehingga penyusunan naskah proposal tugas akhir yang berjudul *"Analisis Kinematika, Dinamika, dan Estimasi Keadaan Optimal *Kalman Filter* untuk *Vision-Based Tracking* pada *Over-Actuated 8-Thruster 6-DOF Vectored AUV"* ini dapat diselesaikan dengan baik. Naskah proposal ini disusun sebagai salah satu persyaratan kurikulum akademik untuk memperoleh gelar Sarjana Teknik (S.T.) pada Program Studi Teknik Mesin, Departemen Teknik Mesin, Fakultas Teknik, Universitas Hasanuddin.

Penyelesaian naskah proposal tugas akhir ini tidak lepas dari bimbingan, arahan, dorongan motivasi, serta bantuan berharga dari berbagai pihak. Oleh karena itu, dengan penuh rasa hormat dan kerendahan hati, penulis menyampaikan terima kasih dan penghargaan yang setinggi-tingginya kepada:

1. **Bapak Prof. Dr. Ir. Jamaluddin Jompa, M.Sc.**, selaku Rektor Universitas Hasanuddin.
2. **Bapak Prof. Dr. Eng. Ir. Muhammad Isran Ramli, S.T., M.T.**, selaku Dekan Fakultas Teknik, Universitas Hasanuddin.
3. **Bapak/Ibu [Nama Ketua Departemen & Gelar]**, selaku Ketua Departemen Teknik Mesin, Fakultas Teknik, Universitas Hasanuddin.
4. **Bapak/Ibu [Nama Ketua Program Studi & Gelar]**, selaku Ketua Program Studi Teknik Mesin, Fakultas Teknik, Universitas Hasanuddin.
5. **Bapak/Ibu [Nama Pembimbing Utama & Gelar]**, selaku Pembimbing Utama, yang senantiasa meluangkan waktu, memberikan bimbingan ilmiah yang sangat berharga, arahan matematis yang mendalam, serta teladan profesionalisme dalam penyusunan penelitian ini.
6. **Bapak/Ibu [Nama Pembimbing Pendamping & Gelar]*, selaku Pembimbing Pendamping, atas segala masukan teknis, telaah kritis, saran konstruktif, dan dukungan moril yang senantiasa membimbing penulis.
7. Seluruh Dosen dan Staf Pengajar di lingkungan Program Studi Teknik Mesin dan Departemen Teknik Mesin Universitas Hasanuddin atas bekal keilmuan, wawasan teknik, dan dedikasi akademis yang telah dicurahkan selama masa perkuliahan.
8. Rekan-rekan mahasiswa dan asisten di Laboratorium Mekatronika dan Robotika atas diskusi teknis, kolaborasi ilmiah, dan kebersamaan dalam eksplorasi teknologi subsea robotics.
9. Teristimewa kepada kedua orang tua tercinta, keluarga besar, dan sanak saudara, atas doa tulus yang tak pernah terputus, cinta kasih tanpa pamrih, pengorbanan, dan dorongan moral serta spiritual yang tak ternilai harganya.

Penulis menyadari sepenuhnya bahwa naskah proposal ini masih memiliki ruang untuk penyempurnaan. Oleh karena itu, saran dan kritik konstruktif sangat diharapkan demi penyempurnaan penelitian ini hingga tahap akhir. Semoga penelitian ini dapat memberikan kontribusi nyata bagi perkembangan ilmu pengetahuan dan teknologi kelautan nasional, khususnya dalam rekayasa robotika bawah air (underwater robotics*).

<br><br>

<table style="width:100%; border:none;">
  <tr>
    <td style="width:50%; border:none;"></td>
    <td style="width:50%; border:none; text-align:center;">
      Makassar, ......................... 2026<br><br>
      Penulis,<br><br><br><br>
      <strong>MUH. RADHI SYAFIQ GHANIM. S</strong>
    </td>
  </tr>
</table>

<div style="page-break-after: always;"></div>

---

<!-- ======================================================================= -->
<!-- ABSTRAK (BAHASA INDONESIA - HALAMAN vi)                                 -->
<!-- ======================================================================= -->

<div align="center">

*Halaman vi

# ABSTRAK

<br>

*MUH. RADHI SYAFIQ GHANIM. S. Analisis Kinematika, Dinamika, dan Estimasi Keadaan Optimal *Kalman Filter* untuk *Vision-Based Tracking* pada Over-Actuated 8-Thruster 6-DOF Vectored AUV* (dibimbing oleh Andi Amijoyo Mochtar, S.T., M.Sc., Ph.D. dan [Nama Pembimbing Pendamping]).

</div>

<br>

*Latar belakang.* Eksplorasi dan pemantauan infrastruktur bawah air menuntut wahana otonom dengan manuver tinggi. Sebagian besar AUV mikro konvensional bekerja dalam kondisi underactuated* (6 pendorong) yang tidak memiliki kendali aktif pada derajat kebebasan *pitch* dan rentan terhadap momen kopling hidrodinamika tidak stabil seperti *Munk moment*. Wahana *over-actuated 8-pendorong mampu menyediakan kendali aktif 6 derajat kebebasan (6-DOF) penuh, namun menghadirkan kompleksitas non-linearitas hidrodinamika Navier-Stokes serta degradasi sensor visual akibat turbiditas air. *Tujuan.* Penelitian ini bertujuan memformulasikan model matematis lengkap kinematika dan dinamika 6-DOF, menyusun matriks alokasi gaya dorong $$6 \times 8$$ berbasis pseudo-inverse* Moore-Penrose, merancang suite Kalman Filter optimal untuk pelacakan target visual dan estimasi dinamika wahana, serta memvalidasi performa sistem melalui integrasi *Software-In-The-Loop* (SITL) dan *Hardware-In-The-Loop (HITL). *Metode.* Kinematika 6-DOF diturunkan melalui grup rotasi $$SO(3)$$ dan kuaternion unit bebas singularitas. Persamaan dinamika non-linear diturunkan berbasis model Fossen, mencakup tensor massa total ($$\mathbf{M}_{RB} + \mathbf{M}_A$$), matriks Coriolis-sentripetal ($$\mathbf{C}_{RB} + \mathbf{C}_A$$), redaman kuadratik Morison, dan vektor pemulih hidrostatis. Redundansi aktuasi diselesaikan melalui alokasi daya dorong minimum. Estimasi keadaan visual menggunakan Kalman Filter diskrit 8D berbasis Continuous White Noise Acceleration* (CWNA) dengan *Mahalanobis distance gating, sedangkan estimasi dinamika menggunakan *Extended Kalman Filter* (EKF) pada *companion computer* Raspberry Pi 4B yang terhubung secara serial MAVLink (50 Hz) dengan *flight controller* Pixhawk 2.4.8 (ArduSub `*vectored_6dof`) dan simulator Gazebo Harmonic/ROS 2. *Hasil yang diharapkan.* Penelitian ini menghasilkan formulasi matematis lengkap, matriks alokasi gaya dorong terverifikasi, serta algoritma penapis Kalman yang mampu mengeliminasi derau deteksi YOLO dan menjaga stabilitas orientasi 6-DOF (pitch-holding*) secara *real-time. *Kesimpulan.* Integrasi pemodelan dinamika 6-DOF first-principles dengan estimasi Kalman Filter optimal memberikan landasan teoretis dan arsitektur mekatronika yang kokoh untuk inspeksi otonom bawah air.

<br>

*Kata kunci:* AUV over-actuated; dinamika 6-DOF; alokasi gaya dorong; Extended Kalman Filter; pelacakan visual YOLO; Hardware-In-The-Loop

<div style="page-break-after: always;"></div>

---

<!-- ======================================================================= -->
<!-- DAFTAR ISI (TABLE OF CONTENTS - HALAMAN vii)                            -->
<!-- ======================================================================= -->

<div align="center">

*Halaman vii*

# DAFTAR ISI

</div>

<br>

<table style="width:100%; border:none; line-height:1.6;">
  <tr>
    <td style="border:none;"><strong>HALAMAN JUDUL</strong></td>
    <td style="border:none; text-align:right;"><strong>i</strong></td>
  </tr>
  <tr>
    <td style="border:none;"><strong>HALAMAN PENGAJUAN</strong></td>
    <td style="border:none; text-align:right;"><strong>ii</strong></td>
  </tr>
  <tr>
    <td style="border:none;"><strong>HALAMAN PENGESAHAN</strong></td>
    <td style="border:none; text-align:right;"><strong>iii</strong></td>
  </tr>
  <tr>
    <td style="border:none;"><strong>PERNYATAAN KEASLIAN DAN PELIMPAHAN HAK CIPTA</strong></td>
    <td style="border:none; text-align:right;"><strong>iv</strong></td>
  </tr>
  <tr>
    <td style="border:none;"><strong>PRAKATA</strong></td>
    <td style="border:none; text-align:right;"><strong>v</strong></td>
  </tr>
  <tr>
    <td style="border:none;"><strong>ABSTRAK</strong></td>
    <td style="border:none; text-align:right;"><strong>vi</strong></td>
  </tr>
  <tr>
    <td style="border:none;"><strong>DAFTAR ISI</strong></td>
    <td style="border:none; text-align:right;"><strong>vii</strong></td>
  </tr>
  <tr>
    <td style="border:none;"><strong>DAFTAR TABEL</strong></td>
    <td style="border:none; text-align:right;"><strong>viii</strong></td>
  </tr>
  <tr>
    <td style="border:none;"><strong>DAFTAR GAMBAR</strong></td>
    <td style="border:none; text-align:right;"><strong>ix</strong></td>
  </tr>
  <tr>
    <td style="border:none;"><strong>DAFTAR LAMPIRAN</strong></td>
    <td style="border:none; text-align:right;"><strong>x</strong></td>
  </tr>
  <tr>
    <td style="border:none;"><strong>DAFTAR SINGKATAN, ISTILAH, DAN LAMBANG</strong></td>
    <td style="border:none; text-align:right;"><strong>xi</strong></td>
  </tr>
  <tr>
    <td colspan="2" style="border:none;"><hr style="border-top:1px solid #000;"></td>
  </tr>
  <tr>
    <td style="border:none;"><strong>BAB I. PENDAHULUAN</strong></td>
    <td style="border:none; text-align:right;"><strong>1</strong></td>
  </tr>
  <tr>
    <td style="border:none; padding-left:20px;">1.1 Latar Belakang</td>
    <td style="border:none; text-align:right;">1</td>
  </tr>
  <tr>
    <td style="border:none; padding-left:20px;">1.2 Rumusan Masalah</td>
    <td style="border:none; text-align:right;">5</td>
  </tr>
  <tr>
    <td style="border:none; padding-left:20px;">1.3 Tujuan Penelitian</td>
    <td style="border:none; text-align:right;">6</td>
  </tr>
  <tr>
    <td style="border:none; padding-left:20px;">1.4 Batasan Masalah</td>
    <td style="border:none; text-align:right;">7</td>
  </tr>
  <tr>
    <td style="border:none; padding-left:20px;">1.5 Manfaat Penelitian</td>
    <td style="border:none; text-align:right;">8</td>
  </tr>


  <tr>
    <td colspan="2" style="border:none;"><br></td>
  </tr>
  <tr>
    <td style="border:none;"><strong>BAB II. TINJAUAN PUSTAKA DAN LANDASAN TEORI</strong></td>
    <td style="border:none; text-align:right;"><strong>12</strong></td>
  </tr>
  <tr>
    <td style="border:none; padding-left:20px;">2.1 Tinjauan Pustaka (State of the Art Penelitian AUV)</td>
    <td style="border:none; text-align:right;">12</td>
  </tr>
  <tr>
    <td style="border:none; padding-left:20px;">2.2 Sistem Koordinat dan Konvensi SNAME</td>
    <td style="border:none; text-align:right;">16</td>
  </tr>
  <tr>
    <td style="border:none; padding-left:20px;">2.3 Penurunan Kinematika 6-DOF dan Matriks Jacobian</td>
    <td style="border:none; text-align:right;">20</td>
  </tr>
  <tr>
    <td style="border:none; padding-left:20px;">2.4 Penurunan Dinamika Hidrodinamika 6-DOF (Persamaan Fossen)</td>
    <td style="border:none; text-align:right;">28</td>
  </tr>
  <tr>
    <td style="border:none; padding-left:20px;">2.5 Alokasi Gaya Dorong Sistem *Over-Actuated* 8-Pendorong</td>
    <td style="border:none; text-align:right;">38</td>
  </tr>
  <tr>
    <td style="border:none; padding-left:20px;">2.6 Teori dan Formulasi Optimal *Kalman Filter* Suite</td>
    <td style="border:none; text-align:right;">44</td>
  </tr>

  <tr>
    <td colspan="2" style="border:none;"><br></td>
  </tr>
  <tr>
    <td style="border:none;"><strong>BAB III. METODOLOGI PENELITIAN</strong></td>
    <td style="border:none; text-align:right;"><strong>59</strong></td>
  </tr>
  <tr>
    <td style="border:none; padding-left:20px;">3.1 Tempat dan Waktu Penelitian</td>
    <td style="border:none; text-align:right;">59</td>
  </tr>
  <tr>
    <td style="border:none; padding-left:20px;">3.2 Diagram Alir Penelitian</td>
    <td style="border:none; text-align:right;">60</td>
  </tr>
  <tr>
    <td style="border:none; padding-left:20px;">3.3 Identifikasi Parameter Fisik dan Hidrodinamika</td>
    <td style="border:none; text-align:right;">62</td>
  </tr>
  <tr>
    <td style="border:none; padding-left:20px;">3.4 Perancangan Arsitektur *Software-In-The-Loop* (SITL)</td>
    <td style="border:none; text-align:right;">66</td>
  </tr>
  <tr>
    <td style="border:none; padding-left:20px;">3.5 Perancangan Arsitektur *Hardware-In-The-Loop* (HITL)</td>
    <td style="border:none; text-align:right;">70</td>
  </tr>
  <tr>
    <td style="border:none; padding-left:20px;">3.6 Prosedur Pengujian dan Evaluasi Kinerja</td>
    <td style="border:none; text-align:right;">74</td>
  </tr>

  <tr>
    <td colspan="2" style="border:none;"><hr style="border-top:1px solid #000;"></td>
  </tr>
  <tr>
    <td style="border:none;"><strong>DAFTAR PUSTAKA</strong></td>
    <td style="border:none; text-align:right;"><strong>80</strong></td>
  </tr>
  <tr>
    <td style="border:none;"><strong>LAMPIRAN</strong></td>
    <td style="border:none; text-align:right;"><strong>83</strong></td>
  </tr>
</table>

<div style="page-break-after: always;"></div>

---

<!-- ======================================================================= -->
<!-- DAFTAR TABEL & DAFTAR GAMBAR (HALAMAN ix & x)                           -->
<!-- ======================================================================= -->

<div align="center">

*Halaman ix*

# DAFTAR TABEL

</div>

<br>

<table style="width:100%; border:none; line-height:1.6;">
  <tr>
    <td style="border:none; width:15%;"><strong>Nomor Urut</strong></td>
    <td style="border:none; width:70%;"><strong>Judul Tabel</strong></td>
    <td style="border:none; width:15%; text-align:right;"><strong>Halaman</strong></td>
  </tr>
  <tr>
    <td style="border:none;">Tabel 1.1</td>
    <td style="border:none;">Perbandingan Karakteristik Wahana *Underactuated* vs. *Over-Actuated*</td>
    <td style="border:none; text-align:right;">4</td>
  </tr>
  <tr>
    <td style="border:none;">Tabel 2.1</td>
    <td style="border:none;">Matriks Sintesis Literatur Terkini (2021–2025) Bidang Dinamika dan Kontrol AUV</td>
    <td style="border:none; text-align:right;">14</td>
  </tr>
  <tr>
    <td style="border:none;">Tabel 2.2</td>
    <td style="border:none;">Notasi dan Konvensi 6 Derajat Kebebasan SNAME (1950)</td>
    <td style="border:none; text-align:right;">17</td>
  </tr>
  <tr>
    <td style="border:none;">Tabel 2.3</td>
    <td style="border:none;">Parameter Fisik dan Inersia Bodi Rigid Over-Actuated 8-Pendorong</td>
    <td style="border:none; text-align:right;">29</td>
  </tr>
  <tr>
    <td style="border:none;">Tabel 2.4</td>
    <td style="border:none;">Koefisien Massa Tambah Hidrodinamika (*Added Mass* Derivatives)</td>
    <td style="border:none; text-align:right;">31</td>
  </tr>
  <tr>
    <td style="border:none;">Tabel 2.5</td>
    <td style="border:none;">Koefisien Redaman Hidrodinamika Linier dan Kuadratik Morison</td>
    <td style="border:none; text-align:right;">34</td>
  </tr>
  <tr>
    <td style="border:none;">Tabel 2.6</td>
    <td style="border:none;">Koordinat Spasial dan Vektor Orientasi Unit 8 Pendorong BLDC Underwater Thruster</td>
    <td style="border:none; text-align:right;">39</td>
  </tr>
  <tr>
    <td style="border:none;">Tabel 3.1</td>
    <td style="border:none;">Daftar Komponen Keras (Hardware) Sistem HITL Subsea dan Topside</td>
    <td style="border:none; text-align:right;">71</td>
  </tr>
</table>

<div style="page-break-after: always;"></div>

<div align="center">

*Halaman x*

# DAFTAR GAMBAR

</div>

<br>

<table style="width:100%; border:none; line-height:1.6;">
  <tr>
    <td style="border:none; width:15%;"><strong>Nomor Urut</strong></td>
    <td style="border:none; width:70%;"><strong>Judul Gambar</strong></td>
    <td style="border:none; width:15%; text-align:right;"><strong>Halaman</strong></td>
  </tr>
  <tr>
    <td style="border:none;">Gambar 1.1</td>
    <td style="border:none;">Diagram Siklus Kolaborasi Antara Pemodelan Fisis dan Gemini Notebook</td>
    <td style="border:none; text-align:right;">3</td>
  </tr>
  <tr>
    <td style="border:none;">Gambar 1.2</td>
    <td style="border:none;">Topologi Terdistribusi Subsea (Raspberry Pi 4B) dan Topside via *Tether* Ethernet</td>
    <td style="border:none; text-align:right;">5</td>
  </tr>
  <tr>
    <td style="border:none;">Gambar 2.1</td>
    <td style="border:none;">Sistem Kerangka Acuan Inersia Bumi (Fn - NED) dan Kerangka Acuan Bodi (Fb - FRD)</td>
    <td style="border:none; text-align:right;">18</td>
  </tr>
  <tr>
    <td style="border:none;">Gambar 2.2</td>
    <td style="border:none;">Konvensi Rotasi Intrinsik Sudut Euler *Yaw*-*Pitch*-*Roll* (z-y-x)</td>
    <td style="border:none; text-align:right;">21</td>
  </tr>
  <tr>
    <td style="border:none;">Gambar 2.3</td>
    <td style="border:none;">Kopling Momen Hidrodinamika Munk (Xu - Yv)ur vr pada Bidang Horizontal</td>
    <td style="border:none; text-align:right;">33</td>
  </tr>
  <tr>
    <td style="border:none;">Gambar 2.4</td>
    <td style="border:none;">Konfigurasi Vektor Geometris 8 Pendorong BLDC Underwater Thruster pada Rangka Over-Actuated 8-Pendorong</td>
    <td style="border:none; text-align:right;">40</td>
  </tr>
  <tr>
    <td style="border:none;">Gambar 2.5</td>
    <td style="border:none;">Struktur Rekursif Predict-Update pada *Discrete Kalman Filter* dan EKF</td>
    <td style="border:none; text-align:right;">46</td>
  </tr>
  <tr>
    <td style="border:none;">Gambar 2.6</td>
    <td style="border:none;">Model Ruang Keadaan 8D Penjejakan *Bounding Box* Kamera Monokuler</td>
    <td style="border:none; text-align:right;">51</td>
  </tr>
  <tr>
    <td style="border:none;">Gambar 3.1</td>
    <td style="border:none;">Diagram Alir Tahapan Penelitian Komprehensif</td>
    <td style="border:none; text-align:right;">61</td>
  </tr>
  <tr>
    <td style="border:none;">Gambar 3.2</td>
    <td style="border:none;">Arsitektur Simulasi *Software-In-The-Loop* (SITL) Sistem AUV</td>
    <td style="border:none; text-align:right;">67</td>
  </tr>
  <tr>
    <td style="border:none;">Gambar 3.3</td>
    <td style="border:none;">Arsitektur Integrasi *Hardware-In-The-Loop* (HITL) Mekatronika AUV</td>
    <td style="border:none; text-align:right;">72</td>
  </tr>
  <tr>
    <td style="border:none;">Gambar 3.4</td>
    <td style="border:none;">Rangka (*Frame*) dan Lambung Tekanan Kustom (*Custom Pressure Hull*) AUV 8-Pendorong</td>
    <td style="border:none; text-align:right;">74</td>
  </tr>
  <tr>
    <td style="border:none;">Gambar 3.5</td>
    <td style="border:none;">Papan Pengendali Penerbangan (*Flight Controller*) Pixhawk 2.4.8</td>
    <td style="border:none; text-align:right;">75</td>
  </tr>
  <tr>
    <td style="border:none;">Gambar 3.6</td>
    <td style="border:none;">Komputer Pendamping (*Companion Computer*) Raspberry Pi 4B</td>
    <td style="border:none; text-align:right;">76</td>
  </tr>
  <tr>
    <td style="border:none;">Gambar 3.7</td>
    <td style="border:none;">Modul Pengendali Kecepatan Elektronik (ESC EMAX BLHeli 30A)</td>
    <td style="border:none; text-align:right;">77</td>
  </tr>
  <tr>
    <td style="border:none;">Gambar 3.8</td>
    <td style="border:none;">Motor Pendorong Bawah Air (*BLDC Underwater Thruster*)</td>
    <td style="border:none; text-align:right;">78</td>
  </tr>
  <tr>
    <td style="border:none;">Gambar 3.9</td>
    <td style="border:none;">Sumber Daya Baterai Li-Po 4S 14.8V 6000 mAh dan Pengisi Daya SKYRC IMAX B6AC V2</td>
    <td style="border:none; text-align:right;">79</td>
  </tr>
</table>

<div style="page-break-after: always;"></div>

---

<!-- ======================================================================= -->
<!-- DAFTAR SINGKATAN, ISTILAH, DAN LAMBANG (HALAMAN xii)                     -->
<!-- ======================================================================= -->

<div align="center">

*Halaman xii

# DAFTAR SINGKATAN, ISTILAH, DAN LAMBANG

</div>

<br>

### 1. Daftar Singkatan

| Singkatan | Kepanjangan / Arti Teknis |
| :--- | :--- |
| *AUV* | Autonomous Underwater Vehicle (Wahana Bawah Air Otonom) |
| *HAUV* | Hovering Autonomous Underwater Vehicle (AUV Mampu Melayang di Kolom Air) |
| *ROV* | Remotely Operated Vehicle (Wahana Bawah Air Kendali Jarak Jauh) |
| *DOF* | Degrees of Freedom (Derajat Kebebasan Spasial) |
| *SNAME* | The Society of Naval Architects and Marine Engineers |
| *NED* | North-East-Down (Sistem Koordinat Inersia Bumi: Utara-Timur-Bawah) |
| *FRD* | Forward-Right-Down (Sistem Koordinat Bodi Wahana: Maju-Kanan-Bawah) |
| *CG* | Center of Gravity (Titik Pusat Massa/Gravitasi Wahana) |
| *CB* | Center of Buoyancy (Titik Pusat Gaya Apung Hidrostatis) |
| *CO* | Center of Origin (Pusat Titik Acuan Kerangka Bodi) |
| *CWNA* | Continuous White Noise Acceleration (Model Stokastik Penjejakan Kinematik) |
| *EKF** | *Extended Kalman Filter (Penapis Kalman Non-Linier) |
| SITL** | *Software-In-The-Loop (Simulasi Fisika Terintegrasi Perangkat Lunak) |
| *HITL* | Hardware-In-The-Loop (Pengujian Terintegrasi Perangkat Keras Riil) |
| *YOLO* | You Only Look Once (Arsitektur Jaringan Saraf Konvolusional Deteksi Objek) |
| *ROS* | Robot Operating System* (*Middleware Komunikasi Robotika) |
| *MAVLink* | Micro Air Vehicle Link (Protokol Telemetri Biner Serial Robotika Otonom) |
| *PWM* | Pulse Width Modulation (Sinyal Modulasi Lebar Pulsa Kendali Motor) |
| *ESC* | Electronic Speed Controller* (Pengendali Kecepatan Motor *Brushless) |
| *IMU* | Inertial Measurement Unit (Unit Pengukuran Inersia: Akselerometer & Giroskop) |
| *DVL* | Doppler Velocity Log* (Sensor Akustik Pengukur Kecepatan Relatif Air) |

<br>

### 2. Daftar Lambang dan Simbol Matematika

| Simbol | Dimensi / Satuan | Definisi Matematis dan Fisik |
| :--- | :---: | :--- |
| $$\mathcal{F}^n$$ | - | Kerangka Acuan Inersia Bumi (*Earth-Fixed NED Frame*) $$\{O_n, x_n, y_n, z_n\}$$ |
| $$\mathcal{F}^b$$ | - | Kerangka Acuan Bergerak Bodi (*Body-Fixed Frame*) $$\{O_b, x_b, y_b, z_b\}$$ |
| $$\boldsymbol{\eta}$$ | $$\mathbb{R}^6$$ | Vektor posisi dan orientasi spasial di $$\mathcal{F}^n$$: $$[x, y, z, \phi, \theta, \psi]^T$$ |
| $$\boldsymbol{\nu}$$ | $$\mathbb{R}^6$$ | Vektor kecepatan linier dan sudut di $$\mathcal{F}^b$$: $$[u, v, w, p, q, r]^T$$ |
| $$\boldsymbol{\tau}$$ | $$\mathbb{R}^6$$ | Vektor gaya dan momen generalisasi di $$\mathcal{F}^b$$: $$[X, Y, Z, K, M, N]^T$$ |
| $$\boldsymbol{\nu}_c$$ | $$\mathbb{R}^6$$ | Vektor kecepatan arus laut fluida pada kerangka bodi $$[u_c, v_c, w_c, 0, 0, 0]^T$$ |
| $$\boldsymbol{\nu}_r$$ | $$\mathbb{R}^6$$ | Vektor kecepatan relatif wahana terhadap fluida: $$\boldsymbol{\nu} - \boldsymbol{\nu}_c$$ |
| $$\mathbf{R}_b^n(\boldsymbol{\eta}_2)$$ | $$SO(3)$$ | Matriks transformasi rotasi ortogonal dari $$\mathcal{F}^b$$ ke $$\mathcal{F}^n$$ |
| $$\mathbf{T}_\Theta(\boldsymbol{\eta}_2)$$ | $$\mathbb{R}^{3 \times 3}$$ | Matriks transformasi kecepatan sudut Euler: $$\dot{\boldsymbol{\eta}}_2 = \mathbf{T}_\Theta \boldsymbol{\nu}_2$$ |
| $$\mathbf{J}(\boldsymbol{\eta}_2)$$ | $$\mathbb{R}^{6 \times 6}$$ | Matriks Jacobian kinematika gabungan: $$\text{diag}[\mathbf{R}_b^n, \mathbf{T}_\Theta]$$ |
| $$\mathbf{q}$$ | $$S^3$$ | Kuaternion unit orientasi empat-dimensi: $$[\eta, \epsilon_1, \epsilon_2, \epsilon_3]^T$$ |
| $$\mathbf{M}_{RB}$$ | $$\mathbb{R}^{6 \times 6}$$ | Tensor massa inersia bodi kaku (*rigid-body mass matrix*) |
| $$\mathbf{M}_A$$ | $$\mathbb{R}^{6 \times 6}$$ | Tensor massa tambah hidrodinamika fluida (*hydrodynamic added mass*) |
| $$\mathbf{M}$$ | $$\mathbb{R}^{6 \times 6}$$ | Tensor massa sistem total gabungan: $$\mathbf{M} = \mathbf{M}_{RB} + \mathbf{M}_A$$ |
| $$\mathbf{C}_{RB}(\boldsymbol{\nu})$$ | $$\mathbb{R}^{6 \times 6}$$ | Matriks Coriolis dan sentripetal bodi kaku |
| $$\mathbf{C}_A(\boldsymbol{\nu}_r)$$ | $$\mathbb{R}^{6 \times 6}$$ | Matriks Coriolis dan sentripetal massa tambah hidrodinamika |
| $$\mathbf{D}(\boldsymbol{\nu}_r)$$ | $$\mathbb{R}^{6 \times 6}$$ | Tensor redaman hidrodinamika gabungan (linier laminar $$\mathbf{D}_L$$ + kuadratik $$\mathbf{D}_{NL}$$) |
| $$\mathbf{g}(\boldsymbol{\eta})$$ | $$\mathbb{R}^6$$ | Vektor gaya dan momen pemulih hidrostatis (gravitasi dan gaya apung) |
| $$GM_T$$ | $$\text{m}$$ | Tinggi metasentris transversal wahana: $$z_g - z_b$$ |
| $$\mathbf{T}_{6 \times 8}$$ | $$\mathbb{R}^{6 \times 8}$$ | Matriks konfigurasi dan alokasi gaya dorong 8 pendorong |
| $$\mathbf{T}_{6 \times 8}^+$$ | $$\mathbb{R}^{8 \times 6}$$ | Matriks *pseudo-inverse* Moore-Penrose: $$\mathbf{T}^T(\mathbf{T}\mathbf{T}^T)^{-1}$$ |
| $$\mathbf{f}$$ | $$\mathbb{R}^8$$ | Vektor gaya dorong individual 8 motor pendorong $$[f_1, f_2, \dots, f_8]^T$$ ($$\text{N}$$) |
| $$\mathbf{x}_{k}$$ | $$\mathbb{R}^8$$ | Vektor keadaan penjejakan visual: $$[x, y, s, r, \dot{x}, \dot{y}, \dot{s}, \dot{r}]^T$$ |
| $$\mathbf{P}_k$$ | $$\mathbb{R}^{8 \times 8}$$ | Matriks kovariansi kesalahan estimasi (*error covariance matrix*) |
| $$\mathbf{K}_k$$ | - | Matriks penguatan optimal Kalman (*optimal Kalman gain*) |
| $$\mathbf{Q}$$ | - | Matriks kovariansi derau proses (*process noise covariance matrix*) |
| $$\mathbf{R}$$ | - | Matriks kovariansi derau pengukuran (*measurement noise covariance matrix*) |
| $$D_M$$ | - | Jarak kuadratis Mahalanobis untuk *outlier innovation gating* |
