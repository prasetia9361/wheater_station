/*
  Monitor Tegangan LiFePO4 ESP32-S3
  Pin: GPIO 2
  R1: 47k Ohm
  R2: 10k Ohm
*/

const int batteryPin = 2;   // GPIO 2
const float R1 = 47000.0;   // 47k
const float R2 = 10000.0;   // 10k

// Faktor Kalibrasi
// Ubah nilai ini jika pembacaan serial monitor berbeda dengan Multimeter
// Jika di serial 12.5V tapi di multimeter 12.8V, naikkan sedikit.
const float calibrationFactor = 1.00; 

void setup() {
  Serial.begin(115200);
  
  // Konfigurasi resolusi ADC (biasanya default 12-bit)
  analogReadResolution(12);
  
  // Attenuation 11dB agar bisa membaca range 0 - 3.1V (approx)
  analogSetAttenuation(ADC_11db);
  
  pinMode(batteryPin, INPUT);
}

void loop() {
  // 1. Ambil Rata-rata Pembacaan (Multisampling) untuk mengurangi noise
  long totalMilliVolts = 0;
  int sampleCount = 20; // Baca 20 kali

  for(int i = 0; i < sampleCount; i++) {
    // analogReadMilliVolts mengembalikan nilai dalam mV (sudah dikalibrasi pabrik)
    totalMilliVolts += analogReadMilliVolts(batteryPin);
    delay(5);
  }
  
  float avgMilliVolts = totalMilliVolts / sampleCount;

  // 2. Hitung Tegangan pada Pin GPIO (Vout)
  float voltageGPIO = avgMilliVolts / 1000.0; // Konversi mV ke Volt

  // 3. Hitung Tegangan Asli Baterai menggunakan Rumus Pembagi Tegangan
  // Vin = Vout * (R1 + R2) / R2
  float voltageBattery = voltageGPIO * ((R1 + R2) / R2);

  // 4. Terapkan faktor kalibrasi manual (finetuning)
  voltageBattery = voltageBattery * calibrationFactor;

  // Tampilkan Hasil
  Serial.print("ADC (Pin): ");
  Serial.print(voltageGPIO);
  Serial.print(" V | Tegangan Baterai: ");
  Serial.print(voltageBattery);
  Serial.println(" V");
  
  // Persentase kasar LiFePO4 (Estimasi Sederhana untuk 4S)
  // 13.6V = 100%, 12.0V = 0% (Kira-kira)
  int percent = map((long)(voltageBattery * 100), 1200, 1360, 0, 100);
  if(percent > 100) percent = 100;
  if(percent < 0) percent = 0;
  
  Serial.print("Kapasitas: ");
  Serial.print(percent);
  Serial.println("%");
  Serial.println("-------------------------");

  delay(2000);
}