import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

   # 3-DoF RRR Planar Robot Sınıfı
class RRRRobot:
    

    def __init__(self, l1=0.5, l2=0.4, l3=0.2):
       
       # Robot bağlantı uzunluklarını tanımlama
        self.l1 = l1
        self.l2 = l2
        self.l3 = l3
# Denavit-Hartenberg dönüşüm matrisi
    def dh_matrix(self, theta, alpha, a, d):
        
        return np.array([
            [np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
            [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
            [0,              np.sin(alpha),                np.cos(alpha),               d],
            [0,              0,                             0,                           1]
        ])
# İleri kinematik hesaplama
    def forward_kinematics(self, q):
       # Mafsal açılarını alarak son efektörün konumunu ve yönelimini hesaplar
        theta1, theta2, theta3 = q

        # DH parametreleri ile dönüşüm matrisleri
        A1 = self.dh_matrix(theta1, 0, 0, 0)
        A2 = self.dh_matrix(theta2, 0, self.l1, 0)
        A3 = self.dh_matrix(theta3, 0, self.l2, 0)

        # Toplam dönüşüm
        T = A1 @ A2 @ A3

        # Konum ve yönelim
        position = T[:3, 3]
        orientation = np.arctan2(T[1, 0], T[0, 0])

        return T, position, orientation

# Ters kinematik hesaplama
    def inverse_kinematics(self, target, orientation=None, config='elbow_up'):
       # Hedef pozisyon ve yönelim için ters kinematik hesaplar
        x, y, z = target
        if abs(z) > 1e-10:
            raise ValueError("Bu robot sadece 2D düzlemde çalışır. Z koordinatı sıfır olmalıdır.")

        # Hedef yönelim (verilmemişse optimize et)
        if orientation is None:
            # Yönelim serbest - en uygun θ3'ü bul
            return self._ik_position_only(target, config)

        # Bilek pozisyonu
        wx = x - self.l3 * np.cos(orientation)
        wy = y - self.l3 * np.sin(orientation)
# Bilek noktasına olan uzaklık
        r = np.sqrt(wx**2 + wy**2)

        # Ulaşılabilirlik kontrolü
        if r > self.l1 + self.l2 or r < abs(self.l1 - self.l2):
            raise ValueError(f"Hedef nokta ulaşılabilir değil. Mesafe: {r:.3f}m")

        # θ2 hesapla (kosinüs teoremi)
        cos_theta2 = (r**2 - self.l1**2 - self.l2**2) / (2 * self.l1 * self.l2)
        cos_theta2 = np.clip(cos_theta2, -1, 1)

        if config == 'elbow_up':
            sin_theta2 = np.sqrt(1 - cos_theta2**2)
        else:  # elbow_down
            sin_theta2 = -np.sqrt(1 - cos_theta2**2)
        #theta2'yi atan2 ile hesapla
        theta2 = np.arctan2(sin_theta2, cos_theta2)

        # θ1 hesapla
        beta = np.arctan2(wy, wx)
        psi = np.arctan2(self.l2 * sin_theta2, self.l1 + self.l2 * cos_theta2)
        theta1 = beta - psi

        # θ3 hesapla
        theta3 = orientation - theta1 - theta2

        return np.array([theta1, theta2, theta3])
    

    # Yalnızca pozisyon için ters kinematik (yönelim serbest)
    def _ik_position_only(self, target, config='elbow_up'):
      
        x, y, _ = target
    # Hedef noktanın uzaklığı
        r = np.sqrt(x**2 + y**2)

        # Ulaşılabilirlik kontrolü
        max_reach = self.l1 + self.l2 + self.l3
        min_reach = abs(self.l1 - self.l2 - self.l3)

        if r > max_reach or r < min_reach:
            raise ValueError(f"Hedef nokta ulaşılabilir değil. Mesafe: {r:.3f}m")

        # θ2 hesapla (bilek serbest)
        cos_theta2 = (r**2 - self.l1**2 - self.l2**2) / (2 * self.l1 * self.l2)
        cos_theta2 = np.clip(cos_theta2, -1, 1)

        if config == 'elbow_up':
            sin_theta2 = np.sqrt(1 - cos_theta2**2)
        else:
            sin_theta2 = -np.sqrt(1 - cos_theta2**2)

        theta2 = np.arctan2(sin_theta2, cos_theta2)

        # θ1 hesapla
        beta = np.arctan2(y, x)
        psi = np.arctan2(self.l2 * sin_theta2, self.l1 + self.l2 * cos_theta2)
        theta1 = beta - psi

        # θ3'ü 0 olarak al (yönelim serbest)
        theta3 = 0

        return np.array([theta1, theta2, theta3])


# Jakobiyen matrisi hesaplama
    def jacobian(self, q):
     
        theta1, theta2, theta3 = q

        # Konum
        _, p, _ = self.forward_kinematics(q)

        # Jakobiyen elemanları
        J11 = -self.l1*np.sin(theta1) - self.l2*np.sin(theta1+theta2) - self.l3*np.sin(theta1+theta2+theta3)
        J12 = -self.l2*np.sin(theta1+theta2) - self.l3*np.sin(theta1+theta2+theta3)
        J13 = -self.l3*np.sin(theta1+theta2+theta3)

        J21 = self.l1*np.cos(theta1) + self.l2*np.cos(theta1+theta2) + self.l3*np.cos(theta1+theta2+theta3)
        J22 = self.l2*np.cos(theta1+theta2) + self.l3*np.cos(theta1+theta2+theta3)
        J23 = self.l3*np.cos(theta1+theta2+theta3)

        J = np.array([[J11, J12, J13],
                      [J21, J22, J23]])

        return J

# Singülerlik kontrolü
    def check_singularity(self, q, threshold=1e-6):
       
        J = self.jacobian(q)
        J_sub = J[:, :2]  # 2x2 alt matris

        det = np.linalg.det(J_sub @ J_sub.T)
        is_singular = abs(det) < threshold

        return is_singular, det

    def plot_robot(self, q, ax=None, color='blue'):
        """
        Robot konfigürasyonunu çizmek için yardımcı fonksiyon

        Parametreler:
            q: Mafsal açıları
            ax: Matplotlib axis
        """
        theta1, theta2, theta3 = q

        # Eklem pozisyonlarını hesapla
        p0 = np.array([0, 0])
        p1 = np.array([self.l1 * np.cos(theta1), self.l1 * np.sin(theta1)])
        p2 = p1 + np.array([self.l2 * np.cos(theta1+theta2), self.l2 * np.sin(theta1+theta2)])
        p3 = p2 + np.array([self.l3 * np.cos(theta1+theta2+theta3), self.l3 * np.sin(theta1+theta2+theta3)])

        points = np.array([p0, p1, p2, p3])

        if ax is None:
            _, ax = plt.subplots(figsize=(8, 8))

        # Robot kolu çiz
        ax.plot(points[:, 0], points[:, 1], 'o-', color=color, linewidth=2, markersize=8)
        ax.plot(p3[0], p3[1], 'r*', markersize=12, label='End-Effector')

        # Eklemleri işaretle
        ax.plot(p0[0], p0[1], 'ks', markersize=10, label='Base')

        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_title(f'Robot Konfigürasyonu: θ=[{np.rad2deg(q[0]):.1f}°, {np.rad2deg(q[1]):.1f}°, {np.rad2deg(q[2]):.1f}°]')
        ax.grid(True, alpha=0.3)
        ax.axis('equal')

        return ax



def main():

    print("3-DoF RRR Planar Robot - İleri ve Ters Kinematik Analizi")
    print("="*60)

    # Robot nesnesi oluştur
    robot = RRRRobot(l1=0.5, l2=0.4, l3=0.3)

    # 1. İLERİ KİNEMATİK TESTİ
    print("\n" + "="*60)
    print("1. İLERİ KİNEMATİK")
    print("="*60)

    q_test = np.array([0, np.pi/4, np.pi/2])  # [0°, 45°, 90°]
    print(f"\nTest mafsal açıları: θ1={np.rad2deg(q_test[0]):.1f}°, θ2={np.rad2deg(q_test[1]):.1f}°, θ3={np.rad2deg(q_test[2]):.1f}°")
# İleri kinematik hesaplama
    T, pos, orient = robot.forward_kinematics(q_test)
    print(f"\nSon efektör konumu: x={pos[0]:.3f} m, y={pos[1]:.3f} m, z={pos[2]:.1f} m")
    print(f"Son efektör yönelimi: {np.rad2deg(orient):.1f}°")

    # 2. TERS KİNEMATİK TESTİ
    print("\n" + "="*60)
    print("2. TERS KİNEMATİK")
    print("="*60)

    # Hedef nokta belirle
    target = np.array([0.8, 0.2, 0.0])
    print(f"\nHedef nokta: [{target[0]}, {target[1]}, {target[2]}]")

    try:
        # Elbow-up çözümü
        q_ik_up = robot.inverse_kinematics(target, config='elbow_up')
        print(f"\nElbow-up çözümü:")
        print(f"  θ1 = {np.rad2deg(q_ik_up[0]):.2f}°")
        print(f"  θ2 = {np.rad2deg(q_ik_up[1]):.2f}°")
        print(f"  θ3 = {np.rad2deg(q_ik_up[2]):.2f}°")

        # Elbow-down çözümü
        q_ik_down = robot.inverse_kinematics(target, config='elbow_down')
        print(f"\nElbow-down çözümü:")
        print(f"  θ1 = {np.rad2deg(q_ik_down[0]):.2f}°")
        print(f"  θ2 = {np.rad2deg(q_ik_down[1]):.2f}°")
        print(f"  θ3 = {np.rad2deg(q_ik_down[2]):.2f}°")

        # Doğrulama
        _, pos_up, _ = robot.forward_kinematics(q_ik_up)
        _, pos_down, _ = robot.forward_kinematics(q_ik_down)

        print(f"\nDoğrulama (elbow-up): ulaşılan nokta [{pos_up[0]:.3f}, {pos_up[1]:.3f}]")
        print(f"Doğrulama (elbow-down): ulaşılan nokta [{pos_down[0]:.3f}, {pos_down[1]:.3f}]")

    except ValueError as e:
        print(f"IK hatası: {e}")

    # 3. SİNGÜLERLİK ANALİZİ
    print("\n" + "="*60)
    print("3. SİNGÜLERLİK ANALİZİ")
    print("="*60)

    # Singüler konfigürasyon testi
    q_sing = np.array([0, 0, np.pi/2])  # θ2=0 (kol açık)
    is_sing, det = robot.check_singularity(q_sing)

    print(f"\nKonfigürasyon: θ1=0°, θ2=0°, θ3=90°")
    print(f"  Jakobiyen determinant: {det:.2e}")
    print(f"  Singülerlik: {'EVET' if is_sing else 'HAYIR'}")

    q_normal = np.array([0, np.pi/4, np.pi/4])
    is_sing, det = robot.check_singularity(q_normal)

    print(f"\nKonfigürasyon: θ1=0°, θ2=45°, θ3=45°")
    print(f"  Jakobiyen determinant: {det:.2e}")
    print(f"  Singülerlik: {'EVET' if is_sing else 'HAYIR'}")

    # 4. GÖRSELLEŞTİRME
    print("\n" + "="*60)
    print("4. GÖRSELLEŞTİRME")
    print("="*60)

    # Şekil oluştur
    fig, axes = plt.subplots(2, 2, figsize=(14, 12))

    # (a) İleri kinematik test konfigürasyonu
    robot.plot_robot(q_test, ax=axes[0, 0], color='blue')
    axes[0, 0].set_title('İleri Kinematik Testi\nθ=[0°, 45°, 90°]')

    # (b) IK elbow-up çözümü
    if 'q_ik_up' in locals():
        robot.plot_robot(q_ik_up, ax=axes[0, 1], color='green')
        axes[0, 1].set_title(f'IK Elbow-up Çözümü\nHedef: [{target[0]}, {target[1]}]')

    # (c) IK elbow-down çözümü
    if 'q_ik_down' in locals():
        robot.plot_robot(q_ik_down, ax=axes[1, 0], color='orange')
        axes[1, 0].set_title(f'IK Elbow-down Çözümü\nHedef: [{target[0]}, {target[1]}]')

    # (d) Singüler konfigürasyon
    robot.plot_robot(q_sing, ax=axes[1, 1], color='red')
    axes[1, 1].set_title('Singüler Konfigürasyon\nθ=[0°, 0°, 90°]')

    plt.tight_layout()
    plt.savefig('robot_konfigurasyonlari.png', dpi=150, bbox_inches='tight')
    plt.show()

    # 5. ÇALIŞMA ALANI GÖRSELLEŞTİRMESİ
    print("\nÇalışma alanı hesaplanıyor...")

    fig, axes = plt.subplots(1, 2, figsize=(14, 6))

    # (a) Sınırsız çalışma alanı
    print("  Sınırsız mafsal açıları ile çalışma alanı hesaplanıyor...")
    n_samples = 20000
    positions_full = []

    for _ in range(n_samples):
        q_rand = np.random.uniform(-np.pi, np.pi, 3)
        _, pos, _ = robot.forward_kinematics(q_rand)
        positions_full.append(pos[:2])

    positions_full = np.array(positions_full)

    axes[0].scatter(positions_full[:, 0], positions_full[:, 1], s=1, alpha=0.3, c='blue')
    axes[0].set_xlabel('X (m)')
    axes[0].set_ylabel('Y (m)')
    axes[0].set_title(f'Sınırsız Çalışma Alanı\n{n_samples} nokta')
    axes[0].grid(True, alpha=0.3)
    axes[0].axis('equal')

    # Teorik sınırları çiz
    theta = np.linspace(0, 2*np.pi, 100)
    x_inner = 0.2 * np.cos(theta)
    y_inner = 0.2 * np.sin(theta)
    x_outer = 1.2 * np.cos(theta)
    y_outer = 1.2 * np.sin(theta)

    axes[0].plot(x_inner, y_inner, 'r--', linewidth=1, label='Teorik iç sınır (0.2m)')
    axes[0].plot(x_outer, y_outer, 'g--', linewidth=1, label='Teorik dış sınır (1.2m)')
    axes[0].legend()

    # (b) Sınırlı çalışma alanı
    print("  Sınırlı mafsal açıları ile çalışma alanı hesaplanıyor...")
    positions_limited = []

    for _ in range(n_samples):
        q_rand = np.random.uniform(-np.pi/2, np.pi/2, 3)  # Sınırlı açılar
        _, pos, _ = robot.forward_kinematics(q_rand)
        positions_limited.append(pos[:2])

    positions_limited = np.array(positions_limited)

    axes[1].scatter(positions_limited[:, 0], positions_limited[:, 1], s=1, alpha=0.3, c='green')
    axes[1].set_xlabel('X (m)')
    axes[1].set_ylabel('Y (m)')
    axes[1].set_title(f'Sınırlı Çalışma Alanı\nθ∈[-90°, 90°], {n_samples} nokta')
    axes[1].grid(True, alpha=0.3)
    axes[1].axis('equal')

    # Sınırlı alan için teorik sınırları çiz
    axes[1].plot(x_outer, y_outer, 'r--', linewidth=1, label='Maksimum erişim (1.2m)')
    axes[1].legend()

    plt.tight_layout()
    plt.savefig('calisma_alani_analizi.png', dpi=150, bbox_inches='tight')
    plt.show()

    print("\n" + "="*60)
    print("ANALİZ TAMAMLANDI")
    print("="*60)

if __name__ == "__main__":
    main()