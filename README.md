# 3-DoF RRR Planar Robot Kolunun İleri ve Ters Kinematik Analizi


Bu proje, YZR502 Robot Kinematiği dersi kapsamında hazırlanmıştır. Üç serbestlik dereceli (3-DoF) düzlemsel RRR (Revolute-Revolute-Revolute) tipi bir robot kolunun ileri ve ters kinematik analizini içermektedir.

## 📋 İçindekiler
- [Proje Hakkında](#proje-hakkında)
- [Robot Seçimi](#robot-seçimi)
- [Özellikler](#özellikler)
- [Kurulum](#kurulum)
- [Kullanım](#kullanım)
- [Örnek Çalıştırma](#örnek-çalıştırma)
- [Sonuçlar](#sonuçlar)
- [Kullanılan Kaynaklar](#kullanılan-kaynaklar)
- [Lisans](#lisans)

## 🎯 Proje Hakkında

Bu projenin amacı:
- 3-DoF RRR planar robot kolunun Denavit-Hartenberg (DH) parametrelerini belirlemek
- İleri kinematik (FK) denklemlerini türetmek ve uygulamak
- Ters kinematik (IK) problemini analitik olarak çözmek
- Robotun çalışma alanını (workspace) görselleştirmek
- Singülerlik analizi yapmak

## 🤖 Robot Seçimi

### Robot Tipi: **RRR Planar (3-DoF)**

**Bağlantı Uzunlukları:**
- L₁ = 0.5 m (Birinci bağlantı)
- L₂ = 0.4 m (İkinci bağlantı)
- L₃ = 0.3 m (Üçüncü bağlantı)

**Mafsal Limitleri:**
- θᵢ ∈ [-π, π] radyan (sınırsız)
- θᵢ ∈ [-π/2, π/2] radyan (sınırlı)

**DH Parametreleri:**

| Eklem i | αᵢ₋₁ (rad) | aᵢ₋₁ (m) | dᵢ (m) | θᵢ (rad) |
|---------|------------|----------|--------|----------|
| 1 | 0 | 0 | 0 | θ₁ |
| 2 | 0 | 0.5 | 0 | θ₂ |
| 3 | 0 | 0.4 | 0 | θ₃ |

## ✨ Özellikler

- ✅ İleri kinematik hesaplama (analitik)
- ✅ Ters kinematik hesaplama (elbow-up/elbow-down)
- ✅ Jakobiyen matrisi hesaplama
- ✅ Singülerlik analizi
- ✅ Çalışma alanı görselleştirme 
- ✅ Robot konfigürasyonu çizimi
- ✅ Yörünge oluşturma
- ✅ Performans analizi

## 🔧 Kurulum

### Gereksinimler
- Python 3.8 veya üzeri
- pip paket yöneticisi

### Adımlar

1. Repository'yi klonlayın:
```bash
git clone https://github.com/kullaniciadi/RRR_ROBOT.git
cd RRR_ROBOT
