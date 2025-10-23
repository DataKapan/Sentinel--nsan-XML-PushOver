# -*- coding: utf-8 -*-

# ==============================================================================
# Gerekli Kütüphaneler ve Temel Ayarlar
# ==============================================================================
import os
import getpass
import logging
import time
import asyncio
import json
import shutil
import traceback
import smtplib
import socket
from datetime import datetime
from email.mime.text import MIMEText
from email.mime.multipart import MIMEMultipart
from ftplib import FTP

# TensorFlow Lite ve Görüntü İşleme
import numpy as np
from PIL import Image
import tflite_runtime.interpreter as tflite

# Donanım Arayüzleri
import serial
import RPi.GPIO as GPIO

# Ağ İletişimi
import httpx

# Temel dizin (scriptin çalıştığı yer)
APP_DIR = os.path.dirname(os.path.abspath(__file__))
BASE_DIR = os.path.expanduser('~/sentinel') # Ana veri klasörü

# ==============================================================================
# Logging Kurulumu
# ==============================================================================
LOG_FILE_PATH = os.path.join(BASE_DIR, "logs/sentinel.log") # Default log path

def setup_logging(log_path):
    """Logging yapılandırmasını ayarlar."""
    os.makedirs(os.path.dirname(log_path), exist_ok=True)
    for handler in logging.root.handlers[:]:
        try: handler.close()
        except Exception: pass
        logging.root.removeHandler(handler)
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(levelname)s - [%(filename)s:%(lineno)d] - %(message)s',
        handlers=[
            logging.FileHandler(log_path, mode='a', encoding='utf-8'),
            logging.StreamHandler()
        ],
        force=True
    )
    logging.getLogger("httpx").setLevel(logging.WARNING)
    logging.getLogger("httpcore").setLevel(logging.WARNING)
    logging.getLogger("urllib3").setLevel(logging.WARNING)

def log_and_print(message, level=logging.INFO): logging.log(level, message)

# ==============================================================================
# SensorReader Sınıfı (Radar)
# ==============================================================================
class SensorReader:
    def __init__(self, config):
        self.port = config.get('PORT', '/dev/ttyAMA0'); self.baudrate = config.get('BAUDRATE', 115200); self.presence_pin = config.get('PRESENCE_GPIO_PIN')
        self.ser = None; self.last_distance = None
        if self.presence_pin is not None:
            try: GPIO.setmode(GPIO.BCM); GPIO.setup(self.presence_pin, GPIO.IN); log_and_print(f"GPIO {self.presence_pin} ayarlandı.")
            except Exception as e: log_and_print(f"GPIO {self.presence_pin} ayarlanamadı: {e}", logging.ERROR); self.presence_pin = None
        else: log_and_print("GPIO varlık pini tanımlanmamış.", logging.WARNING)
        self.uart_config = {'parity': serial.PARITY_NONE, 'stopbits': serial.STOPBITS_ONE, 'bytesize': serial.EIGHTBITS, 'timeout': 0.1}
        self.calibration_cmd = bytes.fromhex('FDFCFBFA0800120000000064000004030201'); self.read_cmd = bytes.fromhex('FDFCFBFA0800120000006400000004030201')

    def parse_distance(self, data):
        try: data_str = data.decode('ascii', errors='ignore'); return float(data_str.split('Range')[-1].strip().split()[0]) if 'Range' in data_str else None
        except Exception as e: log_and_print(f"Radar parse hatası: {e}", logging.WARNING); return None

    async def initialize(self):
        try:
            self.ser = serial.Serial(port=self.port, baudrate=self.baudrate, **self.uart_config); self.ser.write(self.calibration_cmd)
            log_and_print("Radar kalibrasyon komutu gönderildi."); await asyncio.sleep(1)
            if self.ser.in_waiting > 0: self.ser.read(self.ser.in_waiting)
            log_and_print(f"Radar UART ({self.port}) başlatıldı."); return True
        except serial.SerialException as e: log_and_print(f"Kritik Hata: Radar UART ({self.port}) başlatılamadı: {e}", logging.CRITICAL); self.ser = None; return False
        except Exception as e: log_and_print(f"Radar başlatma hatası: {e}", logging.ERROR); return False

    async def read_presence_and_distance(self):
        presence_from_gpio = False; distance = None; current_presence = False
        if self.presence_pin is not None:
             try: presence_from_gpio = bool(GPIO.input(self.presence_pin))
             except RuntimeError:
                 try: GPIO.setup(self.presence_pin, GPIO.IN); presence_from_gpio = bool(GPIO.input(self.presence_pin))
                 except Exception as e_gpio: log_and_print(f"GPIO tekrar ayarlanamadı: {e_gpio}", logging.ERROR)
             except Exception as e: log_and_print(f"GPIO okuma hatası: {e}", logging.WARNING)
        if self.ser:
            try:
                self.ser.write(self.read_cmd); await asyncio.sleep(0.05)
                if self.ser.in_waiting > 0:
                    received = self.ser.read(self.ser.in_waiting); distance = self.parse_distance(received)
                    if distance is not None: self.last_distance = distance
            except serial.SerialException as e:
                log_and_print(f"UART hatası: {e}. Port tekrar açılıyor...", logging.ERROR); self.ser = None; await asyncio.sleep(1)
                try: self.ser = serial.Serial(port=self.port, baudrate=self.baudrate, **self.uart_config); log_and_print("UART tekrar açıldı.", logging.INFO)
                except Exception as e_reopen: log_and_print(f"UART tekrar açılamadı: {e_reopen}", logging.ERROR)
            except Exception as e: log_and_print(f"Radar UART okuma hatası: {e}", logging.ERROR)
        if self.presence_pin is not None: current_presence = presence_from_gpio
        elif distance is not None: current_presence = distance < 600
        else: current_presence = False
        if current_presence: return True, self.last_distance
        else: self.last_distance = None; return False, None

    def cleanup(self):
        try:
            if self.ser and self.ser.is_open: self.ser.close(); log_and_print("UART portu kapatıldı.")
        except Exception as e: log_and_print(f"UART kapatma hatası: {e}", logging.ERROR)

# ==============================================================================
# CameraManager Sınıfı
# ==============================================================================
class CameraManager:
    def __init__(self, save_dir):
        self.save_dir = save_dir; os.makedirs(save_dir, exist_ok=True); self.is_available = os.path.exists("/dev/video0")
        if not self.is_available: log_and_print("UYARI: Kamera donanımı bulunamadı!", logging.WARNING)

    async def capture_photo(self, case_id=None):
        if not self.is_available: return None
        try:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f'); photo_name = f"{case_id}_{timestamp}.jpg" if case_id else f"img_{timestamp}.jpg"
            photo_path = os.path.join(self.save_dir, photo_name)
            process = await asyncio.create_subprocess_exec("libcamera-jpeg", "-o", photo_path, "--width", "1280", "--height", "720", "--nopreview", "-t", "1000", stdout=asyncio.subprocess.DEVNULL, stderr=asyncio.subprocess.PIPE)
            _, stderr = await process.communicate()
            if process.returncode == 0 and os.path.exists(photo_path): log_and_print(f"Fotoğraf çekildi: {os.path.basename(photo_path)}", logging.DEBUG); return photo_path
            else: log_and_print(f"Kamera hatası: {stderr.decode().strip() if stderr else 'Bilinmeyen'}", logging.ERROR); return None
        except Exception as e: log_and_print(f"Fotoğraf çekme hatası: {e}", logging.ERROR); return None

# ==============================================================================
# TFLiteDetector Sınıfı
# ==============================================================================
class TFLiteDetector:
    def __init__(self, model_path, threshold=0.65):
        self.threshold = threshold
        try:
            self.model_path = model_path
            self.interpreter = tflite.Interpreter(self.model_path); self.interpreter.allocate_tensors()
            self.input_details = self.interpreter.get_input_details(); self.output_details = self.interpreter.get_output_details()
            self.height = self.input_details[0]['shape'][1]; self.width = self.input_details[0]['shape'][2]
            log_and_print(f"TFLite modeli yüklendi: {os.path.basename(self.model_path)}")
        except Exception as e: log_and_print(f"KRİTİK HATA: TFLite modeli yüklenemedi: {self.model_path} - {e}", logging.CRITICAL); raise e

    async def process_image(self, image_path):
        try:
            start_time = time.monotonic(); loop = asyncio.get_running_loop()
            image = await loop.run_in_executor(None, lambda: Image.open(image_path).convert('RGB').resize((self.width, self.height)))
            input_data = np.expand_dims(np.array(image, dtype=np.uint8), axis=0)
            self.interpreter.set_tensor(self.input_details[0]['index'], input_data)
            await loop.run_in_executor(None, self.interpreter.invoke)
            scores = self.interpreter.get_tensor(self.output_details[2]['index'])[0]; classes = self.interpreter.get_tensor(self.output_details[1]['index'])[0]
            human_detections = sum(1 for score, class_id in zip(scores, classes) if score > self.threshold and class_id == 0)
            end_time = time.monotonic()
            log_and_print(f"Görüntü işlendi ({os.path.basename(image_path)}). Süre: {end_time - start_time:.2f}s. İnsan: {human_detections > 0}", logging.DEBUG)
            return human_detections > 0, human_detections
        except Exception as e: log_and_print(f"Görüntü işleme hatası ({os.path.basename(image_path)}): {e}", logging.ERROR); return False, 0

# ==============================================================================
# FTPQueueManager Sınıfı
# ==============================================================================
class FTPQueueManager:
    def __init__(self, config, device_id, device_ip, temp_dir, retry_interval):
        self.config = config; self.device_id = device_id; self.device_ip = device_ip
        self.temp_dir = temp_dir; self.upload_status_file = os.path.join(self.temp_dir, "upload_status.json")
        self.last_attempt = 0; self.retry_interval = retry_interval
        os.makedirs(self.temp_dir, exist_ok=True); self.load_upload_status()

    def ftp_connect_with_retry(self, max_retries=3):
        # Blocking function, run in executor
        for attempt in range(max_retries):
            try:
                ftp = FTP(self.config.get('SERVER'), timeout=15); ftp.login(self.config.get('USERNAME'), self.config.get('PASSWORD'))
                return ftp
            except Exception as e: log_and_print(f"FTP bağlantı denemesi {attempt+1}/{max_retries} başarısız: {e}", logging.WARNING); time.sleep(5)
        return None

    def load_upload_status(self):
        try: self.upload_status = json.load(open(self.upload_status_file)) if os.path.exists(self.upload_status_file) else {}
        except Exception as e: log_and_print(f"FTP yükleme durumu okunamadı: {e}", logging.ERROR); self.upload_status = {}

    def save_upload_status(self):
        try: json.dump(self.upload_status, open(self.upload_status_file, 'w'), indent=4)
        except Exception as e: log_and_print(f"FTP yükleme durumu kaydedilemedi: {e}", logging.ERROR)

    def queue_case(self, case_dir):
        if not os.path.isdir(case_dir): return log_and_print(f"FTP kuyruğa ekleme hatası: Dizin bulunamadı {case_dir}", logging.ERROR)
        try:
            case_name = os.path.basename(case_dir); temp_case_dir = os.path.join(self.temp_dir, case_name)
            if not os.path.exists(temp_case_dir): shutil.copytree(case_dir, temp_case_dir)
            self.upload_status[case_name] = {'original_path': case_dir, 'status': 'pending'}; self.save_upload_status()
            log_and_print(f"Vaka FTP kuyruğuna eklendi: {case_name}")
        except Exception as e: log_and_print(f"Vaka FTP kuyruğuna eklenemedi: {e}", logging.ERROR)

    async def process_queue(self):
        if not self.upload_status or not self.config.get('SERVER') or (time.time() - self.last_attempt < self.retry_interval): return
        self.last_attempt = time.time()
        loop = asyncio.get_running_loop()
        ftp = await loop.run_in_executor(None, self.ftp_connect_with_retry)
        if not ftp: return log_and_print("FTP'ye bağlanılamadı, kuyruk işlemi atlanıyor.", logging.WARNING)

        try: await loop.run_in_executor(None, ftp.set_pasv, False)
        except Exception: pass

        processed_something = False
        for case_name, case_info in list(self.upload_status.items()):
            temp_case_dir = os.path.join(self.temp_dir, case_name)
            if not os.path.exists(temp_case_dir): del self.upload_status[case_name]; processed_something = True; continue
            try:
                main_dir = f"Sentinel/{self.device_id}_{self.device_ip}"; date_folder = datetime.now().strftime('%d%m%y')
                await loop.run_in_executor(None, self._ensure_ftp_path, ftp, ['/', main_dir, date_folder, case_name])
                all_success = True
                files_to_upload = [f for f in os.listdir(temp_case_dir) if os.path.isfile(os.path.join(temp_case_dir, f))]
                for filename in files_to_upload:
                    local_path = os.path.join(temp_case_dir, filename)
                    success = await loop.run_in_executor(None, self._upload_file, ftp, local_path, filename)
                    if not success: all_success = False; break
                    await asyncio.sleep(0.01)
                if all_success:
                    log_and_print(f"Vaka FTP'ye başarıyla yüklendi: {case_name}")
                    shutil.rmtree(temp_case_dir)
                    if os.path.exists(case_info.get('original_path', '')):
                         try: shutil.rmtree(case_info['original_path'])
                         except Exception as e_rm: log_and_print(f"Orijinal vaka klasörü silinemedi {case_info['original_path']}: {e_rm}", logging.WARNING)
                    del self.upload_status[case_name]; processed_something = True
                else: log_and_print(f"Vaka yüklemesi tamamlanamadı ({case_name}).", logging.WARNING)
            except Exception as e: log_and_print(f"FTP Vaka işleme hatası ({case_name}): {e}", logging.ERROR)
            await asyncio.sleep(0.1)
        if processed_something: self.save_upload_status()
        try: await loop.run_in_executor(None, ftp.quit)
        except Exception: pass

    def _ensure_ftp_path(self, ftp, path_parts):
        # Blocking, run in executor
        original_path = ftp.pwd()
        try:
            for part in path_parts:
                if not part or part == '/': continue
                try: ftp.cwd(part)
                except Exception:
                    try: ftp.mkd(part); ftp.cwd(part)
                    except Exception as e_mkd: raise e_mkd # Hata dışarı aktarılmalı
        finally:
             try: ftp.cwd(original_path) # Her zaman orijinal yola dönmeye çalış
             except Exception: log_and_print("FTP: Orijinal yola dönülemedi.", logging.WARNING)


    def _upload_file(self, ftp, local_path, remote_filename):
        # Blocking, run in executor
        try:
            with open(local_path, 'rb') as f: ftp.storbinary(f'STOR {remote_filename}', f); return True
        except Exception as e: log_and_print(f"FTP dosya yükleme hatası ({remote_filename}): {e}", logging.ERROR); return False

    async def start_queue_processor(self):
        log_and_print("FTP Kuyruk İşleyici başlatıldı.")
        while True:
            try: await self.process_queue()
            except Exception as e: log_and_print(f"FTP Kuyruk İşleyici hatası: {e}", logging.ERROR)
            await asyncio.sleep(60)

# ==============================================================================
# CaseReporter Sınıfı
# ==============================================================================
class CaseReporter:
    def __init__(self, config):
        self.config = config
        self.enabled = bool(self.config.get('SENDER_EMAIL') and self.config.get('RECIPIENT_EMAILS') and self.config.get('SENDER_PASSWORD'))
        if not self.enabled: log_and_print("E-posta ayarları eksik, e-posta raporlama devre dışı.", logging.WARNING)
        self.html_template="""<!DOCTYPE html><html lang="tr"><head><meta charset="UTF-8"><style>body{{font-family:Arial,sans-serif;margin:20px;color:#333}}.container{{max-width:800px;margin:auto;border:1px solid #ddd;padding:20px;border-radius:8px}}.header{{text-align:center;border-bottom:2px solid #f0f0f0;padding-bottom:15px;margin-bottom:20px}}.header h1{{margin:0;color:#d9534f}}.report-details table{{width:100%;border-collapse:collapse;margin-bottom:20px}}.report-details th,.report-details td{{border:1px solid #ddd;padding:10px;text-align:left}}.report-details th{{background-color:#f9f9f9;width:30%}}.report-section{{margin-bottom:25px}}.report-section h2{{border-bottom:1px solid #eee;padding-bottom:5px;color:#5a5a5a}}.footer{{margin-top:30px;text-align:center;font-size:.9em;color:#888}}</style></head><body><div class="container"><div class="header"><h1>Güvenlik İhlali Raporu</h1></div><div class="report-details"><h2>Olay Detayları</h2><table><tr><th>Vaka Numarası:</th><td>{case_id}</td></tr><tr><th>Cihaz ID:</th><td>{device_id}</td></tr><tr><th>Tarih:</th><td>{date}</td></tr><tr><th>Saat:</th><td>{time}</td></tr></table></div><div class="report-section"><h2>Olay Özeti</h2><p>Belirtilen tarih ve saatte, izlenen alanda bir hareket algılanmış ve yapılan analiz sonucunda bu hareketin bir **insan** kaynaklı olduğu tespit edilmiştir. Olay anında çekilen fotoğraf kanıt olarak sisteme yüklenmiştir.</p></div><div class="report-section"><h2>Tespit Bilgileri</h2><table><tr><th>Analiz Sonucu:</th><td>İNSAN TESPİT EDİLDİ</td></tr><tr><th>Kanıt Fotoğrafı Sayısı:</th><td>{total_photos}</td></tr></table></div><div class="footer"><p>Rapor Oluşturma Zamanı: {report_time}<br>Sentinel Sistem Versiyonu: 2.1</p></div></div></body></html>"""

    def generate_report_content(self, case_data):
        st = datetime.fromtimestamp(case_data['start_time'])
        return {'case_id': case_data['id'], 'device_id': case_data.get('device_id', 'N/A'), 'date': st.strftime('%d-%m-%Y'), 'time': st.strftime('%H:%M:%S'), 'total_photos': len(case_data.get('photos', [])), 'report_time': datetime.now().strftime('%d-%m-%Y %H:%M:%S')}

    async def send_report(self, case_data):
        if not self.enabled: return False
        loop = asyncio.get_running_loop()
        try:
            report_data = self.generate_report_content(case_data); html_content = self.html_template.format(**report_data)
            msg = MIMEMultipart('alternative'); msg['Subject'] = f"🚨 GÜVENLİK UYARISI ({case_data.get('device_id', '')}): İnsan Tespit Edildi! - Vaka #{case_data['id']}"; msg['From'] = self.config['SENDER_EMAIL']; msg['To'] = ', '.join(self.config['RECIPIENT_EMAILS'])
            msg.attach(MIMEText(html_content, 'html'))
            await loop.run_in_executor(None, self._send_smtp_email, msg)
            log_and_print(f"E-posta raporu gönderildi: {case_data['id']}"); return True
        except Exception as e: log_and_print(f"E-posta raporu gönderme hatası: {e}", logging.ERROR); return False

    def _send_smtp_email(self, msg):
        # Blocking SMTP send in executor
        with smtplib.SMTP(self.config['SMTP_SERVER'], self.config['SMTP_PORT'], timeout=15) as server:
            server.starttls(); server.login(self.config['SENDER_EMAIL'], self.config['SENDER_PASSWORD']); server.send_message(msg)

# ==============================================================================
# PushoverClient Sınıfı (Düzeltilmiş)
# ==============================================================================
class PushoverClient:
    def __init__(self, config):
        self.token = config.get('APP_TOKEN')
        self.target_key = config.get('GROUP_KEY') # Sadece Group Key'i alıyoruz
        self.api_url = "https://api.pushover.net/1/messages.json"

        self.enabled = bool(self.token and self.target_key)
        self.client = httpx.AsyncClient(timeout=30.0) if self.enabled else None

        if not self.enabled:
             log_and_print("Pushover ayarları eksik/hatalı (APP_TOKEN veya GROUP_KEY), Pushover devre dışı.", logging.WARNING)

    async def initialize(self):
        if not self.enabled: return False
        try:
            # Token'ın geçerliliğini test etmek için geçersiz bir kullanıcı anahtarı ile doğrulama yap
            response = await self.client.post("https://api.pushover.net/1/users/validate.json", data={"token": self.token, "user": "invalid-test-user-key"})
            # 400 hatası ve belirli bir mesaj bekliyoruz, bu token'ın geçerli olduğunu gösterir.
            if response.status_code == 400 and "user key is invalid" in response.text: # Hata mesajı kontrolü düzeltildi
                 log_and_print("Pushover token doğrulandı (test isteğiyle).")
                 return True
            else:
                 log_and_print(f"Pushover token doğrulanamadı, yanıt: {response.status_code} - {response.text}", logging.ERROR)
                 self.enabled = False; return False
        except Exception as e:
            log_and_print(f"Pushover'a başlangıç bağlantısı kurulamadı: {e}", logging.ERROR)
            self.enabled = False; return False

    async def send_message(self, message, title="Sentinel Uyarısı", priority=0):
        if not self.enabled: return
        payload = {
            "token": self.token,
            "user": self.target_key, # Doğrudan Group Key'i kullan
            "message": message,
            "title": title,
            "priority": priority
        }
        try:
            response = await self.client.post(self.api_url, data=payload) # JSON yerine form data
            if response.status_code != 200:
                # Pushover 4xx hatalarını logla ama programı durdurma
                log_and_print(f"Pushover mesaj gönderilemedi (Hedef: {self.target_key[:5]}...): HTTP {response.status_code} - {response.text}", logging.WARNING)
        except httpx.RequestError as e: log_and_print(f"Pushover mesaj gönderme isteği hatası: {e}", logging.ERROR)
        except Exception as e: log_and_print(f"Pushover mesaj gönderme sırasında beklenmedik hata: {e}", logging.ERROR)

    async def send_photo(self, photo_path, message="İnsan Tespiti", title="Sentinel Uyarısı"):
        if not self.enabled: return
        try:
            # --- DOSYA OKUMA DÜZELTİLDİ ---
            # Dosyayı okuyup içeriğini al (blocking ama basit)
            with open(photo_path, 'rb') as pf:
                photo_bytes = pf.read()
            # --- DÜZELTME SONU ---

            files = {'attachment': (os.path.basename(photo_path), photo_bytes, 'image/jpeg')}
            payload = {
                "token": self.token,
                "user": self.target_key, # Doğrudan Group Key'i kullan
                "message": message,
                "title": title
            }

            response = await self.client.post(self.api_url, data=payload, files=files) # Multipart form data

            if response.status_code != 200:
                log_and_print(f"Pushover fotoğraf gönderilemedi (Hedef: {self.target_key[:5]}...): HTTP {response.status_code} - {response.text}", logging.WARNING)

        except FileNotFoundError: log_and_print(f"Pushover fotoğraf dosyası bulunamadı: {photo_path}", logging.ERROR)
        except httpx.RequestError as e: log_and_print(f"Pushover fotoğraf gönderme isteği hatası: {e}", logging.ERROR)
        except Exception as e: log_and_print(f"Pushover fotoğraf gönderme sırasında beklenmedik hata: {e}", logging.ERROR)

# ==============================================================================
# MilestoneClient Sınıfı
# ==============================================================================
class MilestoneClient:
    def __init__(self, config):
        self.enabled = config.get("ENABLED", False)
        self.ip = config.get("IP"); self.port = config.get("PORT"); self.zone_name = config.get("ZONE_NAME", "DEFAULT_ZONE")
        if self.enabled and (not self.ip or not self.port): log_and_print("Milestone etkin ama IP/Port eksik!", logging.WARNING); self.enabled = False

    async def trigger_event(self, device_id):
        if not self.enabled: return False
        msg = f"İnsan Tespiti - Cihaz: {device_id}"; xml = f"<event><source>DatakapanSistemi</source><externalid>Bolge_Tetigi</externalid><message>{msg}</message><key>{self.zone_name}</key></event>"
        payload = xml.encode('utf-8') + b'\r\n'; log_and_print(f"Milestone -> {self.zone_name}", logging.DEBUG) # Log seviyesi DEBUG'a düşürüldü
        writer = None
        try:
            reader, writer = await asyncio.wait_for(asyncio.open_connection(self.ip, self.port), timeout=3.0)
            writer.write(payload); await writer.drain(); log_and_print(f"✅ Milestone '{self.zone_name}' tetiklemesi gönderildi.", logging.INFO)
            return True
        except asyncio.TimeoutError: log_and_print(f"❌ HATA: Milestone bağlantısı zaman aşımına uğradı.", logging.ERROR); return False
        except ConnectionRefusedError: log_and_print(f"❌ HATA: Milestone bağlantısı reddedildi ({self.ip}:{self.port}).", logging.ERROR); return False
        except Exception as e: log_and_print(f"❌ HATA: Milestone tetiklemesi başarısız: {e}", logging.ERROR); return False
        finally:
             if writer:
                 try: writer.close(); await writer.wait_closed()
                 except Exception: pass

# ==============================================================================
# Ana Sistem Sınıfı (SentinelSystem)
# ==============================================================================
class SentinelSystem:
    def __init__(self, config):
        self.config = config; self.system_config = config.get('SYSTEM', {})
        self.device_id = config.get('DEVICE_ID', f"UNKNOWN_{socket.gethostname()}")
        self.photo_dir = os.path.join(BASE_DIR, self.system_config.get('PHOTO_DIR_RELATIVE', 'photos'))
        self.temp_dir = os.path.join(BASE_DIR, self.system_config.get('TEMP_DIR_RELATIVE', 'temp'))
        model_path = os.path.join(BASE_DIR, self.system_config.get('MODEL_PATH_RELATIVE', 'model/detect.tflite'))
        os.makedirs(self.photo_dir, exist_ok=True); os.makedirs(self.temp_dir, exist_ok=True); os.makedirs(os.path.dirname(model_path), exist_ok=True, mode=0o755)
        try: s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.connect(("8.8.8.8", 80)); self.device_ip = s.getsockname()[0]; s.close()
        except: self.device_ip = "127.0.0.1"
        self.sensor = SensorReader(config.get('SENSOR', {}))
        self.camera = CameraManager(self.photo_dir)
        try: self.detector = TFLiteDetector(model_path, self.system_config.get('DETECTION_THRESHOLD', 0.65))
        except Exception: log_and_print("Kritik: TFLiteDetector başlatılamadı.", logging.CRITICAL); raise
        self.pushover = PushoverClient(config.get('PUSHOVER', {}))
        self.ftp_manager = FTPQueueManager(config.get('FTP', {}), self.device_id, self.device_ip, self.temp_dir, self.system_config.get('FTP_RETRY_INTERVAL_SECONDS', 300))
        self.case_reporter = CaseReporter(config.get('EMAIL', {}))
        self.milestone_client = MilestoneClient(config.get('MILESTONE', {}))
        self.current_case = None; self.motion_being_processed = False; self.is_online = True; self.pending_notifications = []
        self.offline_reboot_counter = 0 # İnternet kesintisi sayacı

    async def check_internet_connection(self):
        try: await asyncio.get_running_loop().getaddrinfo('one.one.one.one', 53, proto=socket.IPPROTO_TCP); return True
        except Exception: return False

    async def periodic_internet_check(self):
        interval = self.system_config.get('INTERNET_CHECK_INTERVAL_SECONDS', 300)
        reboot_threshold = 2
        log_and_print(f"Periyodik internet kontrolü başlatıldı (Her {interval} saniyede bir). Offline kalınırsa {reboot_threshold * interval / 60:.0f}dk sonra reboot.")
        await asyncio.sleep(15)
        while True:
            is_currently_online = await self.check_internet_connection()
            if is_currently_online:
                self.offline_reboot_counter = 0
                if not self.is_online:
                    self.is_online = True; log_and_print("İNTERNET BAĞLANTISI GERİ GELDİ.", logging.INFO)
                    status_update_msg = f"🟢 Cihaz Tekrar Çevrimiçi: {self.device_id}"
                    await self.pushover.send_message(status_update_msg, title="Sentinel Bağlantı Durumu")
                    await self.process_pending_notifications()
            else:
                self.offline_reboot_counter += 1
                if self.is_online:
                    self.is_online = False; log_and_print("İNTERNET BAĞLANTISI KESİLDİ.", logging.WARNING)
                    status_update_msg = f"🔴 Cihaz Çevrimdışı Oldu: {self.device_id}"
                    asyncio.create_task(self.pushover.send_message(status_update_msg, title="Sentinel Bağlantı Durumu"))
                else:
                     log_and_print(f"İnternet hala yok (Sayaç: {self.offline_reboot_counter}/{reboot_threshold}).", logging.WARNING)
                     if self.offline_reboot_counter >= reboot_threshold:
                         log_and_print(f"İnternet {reboot_threshold * interval / 60:.0f} dakikadır yok. Cihaz yeniden başlatılıyor...", logging.CRITICAL)
                         reboot_msg = f"⚠️ Cihaz ({self.device_id}) internet yokluğu nedeniyle yeniden başlatılıyor..."
                         asyncio.create_task(self.pushover.send_message(reboot_msg, title="Sentinel Yeniden Başlatma", priority=1))
                         await asyncio.sleep(5)
                         log_and_print("Yeniden başlatma komutu: sudo reboot", logging.CRITICAL)
                         # Güvenli kapatma için systemd kullanmayı dene
                         try: os.system('sudo systemctl reboot')
                         except Exception: os.system('sudo reboot') # Fallback
                         await asyncio.sleep(60)
            await asyncio.sleep(interval)

    async def process_pending_notifications(self):
        if not self.pending_notifications or not self.is_online: return
        log_and_print(f"{len(self.pending_notifications)} adet birikmiş bildirim gönderiliyor...")
        await self.pushover.send_message(f"ℹ️ ({self.device_id}) Bağlantı geri geldi. {len(self.pending_notifications)} olay tespit edildi.", title="Sentinel Bildirim Özeti")
        processed_notifications = []
        for notification in list(self.pending_notifications):
            try:
                await self.pushover.send_message(notification['message'], title=notification['title'], priority=1)
                await self.pushover.send_photo(notification['photo'], message=notification['message'], title=notification['title'])
                processed_notifications.append(notification); await asyncio.sleep(3)
            except Exception as e: log_and_print(f"Birikmiş bildirim gönderilirken hata: {e}", logging.WARNING); break
        self.pending_notifications = [n for n in self.pending_notifications if n not in processed_notifications]
        if not self.pending_notifications: log_and_print("Birikmiş bildirimler gönderildi.")
        else: log_and_print(f"{len(self.pending_notifications)} bildirim hala beklemede.")

    async def initialize(self):
        sensor_ok = await self.sensor.initialize(); pushover_ok = await self.pushover.initialize()
        if not sensor_ok: return False
        if not pushover_ok: log_and_print("Uyarı: Pushover başlatılamadı!", logging.WARNING)
        self.is_online = await self.check_internet_connection()
        log_and_print(f"Başlangıç internet durumu: {'Çevrimiçi' if self.is_online else 'Çevrimdışı'}")
        asyncio.create_task(self.periodic_internet_check())
        if self.config.get('FTP') and self.config['FTP'].get('SERVER'): asyncio.create_task(self.ftp_manager.start_queue_processor())
        else: log_and_print("FTP ayarları eksik, FTP Kuyruk İşleyici başlatılmadı.", logging.WARNING)
        log_and_print(f"Sentinel sistemi başlatıldı (Cihaz: {self.device_id})")
        if pushover_ok and self.is_online: await self.pushover.send_message(f"🟢 Cihaz Aktif: {self.device_id} (İnsan Tespit Modu)", title="Sentinel Başlatıldı")
        elif not self.is_online: log_and_print("İnternet yok, başlangıç bildirimi atlandı.", logging.WARNING)
        return True

    def start_case(self):
        case_id = datetime.now().strftime('case_%Y%m%d_%H%M%S')
        case_dir = os.path.join(self.photo_dir, case_id)
        os.makedirs(case_dir, exist_ok=True)
        self.current_case = {'id': case_id, 'dir': case_dir, 'start_time': time.time(), 'photos': [], 'device_id': self.device_id, 'device_ip': self.device_ip}
        log_and_print(f"Hareket algılandı, yeni vaka: {case_id}", logging.DEBUG)

    async def discard_case(self):
        if self.current_case:
            case_dir = self.current_case['dir']
            log_and_print(f"İnsan tespit edilmedi veya kamera hatası. Vaka {self.current_case['id']} siliniyor.", logging.DEBUG)
            try: shutil.rmtree(case_dir)
            except Exception as e: log_and_print(f"Vaka dizini silinirken hata oluştu ({case_dir}): {e}", logging.ERROR)
            self.current_case = None

    async def process_human_detection(self):
        if not self.current_case: return
        case_id = self.current_case['id']; photo_with_human = self.current_case['photos'][-1] if self.current_case['photos'] else None
        message_text = f"🚨 İNSAN TESPİT EDİLDİ! ({self.device_id})\nVaka ID: {case_id}"
        title = f"Sentinel İnsan Uyarısı ({self.device_id})"
        log_and_print(f"🚨 İNSAN TESPİT EDİLDİ! Vaka {case_id} raporlanıyor.")
        current_case_copy = self.current_case.copy()
        if os.path.isdir(current_case_copy['dir']): self.ftp_manager.queue_case(current_case_copy['dir'])
        else: log_and_print(f"FTP'ye eklenemedi: Vaka dizini bulunamadı {current_case_copy['dir']}", logging.ERROR)
        self.current_case = None
        if photo_with_human:
            if self.is_online:
                log_and_print(f"Online: Vaka {case_id} bildirimleri gönderiliyor...")
                tasks = [self.pushover.send_message(message_text, title=title, priority=1)]
                tasks.append(self.pushover.send_photo(photo_with_human, message=message_text, title=title))
                if self.case_reporter.enabled: tasks.append(self.case_reporter.send_report(current_case_copy))
                if self.milestone_client.enabled: tasks.append(self.milestone_client.trigger_event(self.device_id))
                await asyncio.gather(*tasks, return_exceptions=True)
            else:
                log_and_print(f"Offline: Vaka {case_id} bildirimi Pushover kuyruğuna ekleniyor.", logging.WARNING)
                self.pending_notifications.append({'message': message_text, 'title': title, 'photo': photo_with_human, 'case_data': current_case_copy})
        else: log_and_print(f"İnsan tespiti ({case_id}) için kanıt fotoğrafı yok, bildirim gönderilmedi/kuyruğa eklenmedi.", logging.WARNING)
        log_and_print(f"Vaka {case_id} işlemleri tamamlandı.")

    async def run(self):
        if not await self.initialize(): log_and_print("Sistem başlatılamadı. Çıkılıyor.", logging.CRITICAL); return
        calibration_time = self.system_config.get('SENSOR_CALIBRATION_SECONDS', 30)
        log_and_print(f"Sensör kalibrasyonu için {calibration_time} saniye bekleniyor..."); await asyncio.sleep(calibration_time)
        log_and_print("Sistem aktif.");
        
        while True:
            try:
                presence, _ = await self.sensor.read_presence_and_distance()
                if presence and not self.motion_being_processed:
                    self.motion_being_processed = True; self.start_case()
                    human_found = False; camera_error_notified = False; photo_captured = False
                    for i in range(5):
                        if not self.current_case: break
                        log_and_print(f"Fotoğraf {i+1}/5 çekiliyor...", logging.DEBUG)
                        photo_path_raw = await self.camera.capture_photo(self.current_case['id'])
                        if photo_path_raw and os.path.exists(photo_path_raw):
                            photo_captured = True
                            if self.current_case: self.current_case['photos'].append(photo_path_raw)
                            else: break
                            is_human, _ = await self.detector.process_image(photo_path_raw)
                            if is_human:
                                if self.current_case: human_found = True; await self.process_human_detection(); break
                                else: log_and_print("İnsan bulundu ama vaka kapatılmıştı.", logging.WARNING); break
                        else:
                            if not camera_error_notified:
                                error_text = f"⚠️ SİSTEM UYARISI ({self.device_id}): Kamera Hatası! Fotoğraf çekilemiyor."
                                if self.is_online: await self.pushover.send_message(error_text, title="Sentinel Kamera Hatası")
                                camera_error_notified = True
                        if not human_found and i < 4: await asyncio.sleep(2)
                    
                    if not human_found and self.current_case: await self.discard_case()

                    cooldown = self.system_config.get('CASE_COOLDOWN_SECONDS', 1)
                    log_and_print(f"Vaka döngüsü tamamlandı. {cooldown}s soğuma.", logging.DEBUG); await asyncio.sleep(cooldown)
                    self.motion_being_processed = False
                    log_and_print("Sistem yeniden hareket algılamaya hazır.", logging.DEBUG)
                else: await asyncio.sleep(0.1)
            except Exception as e:
                log_and_print(f"Ana döngüde kritik hata: {e}\n{traceback.format_exc()}", level=logging.CRITICAL)
                if self.current_case: await self.discard_case()
                self.motion_being_processed = False
                await asyncio.sleep(10)

# ==============================================================================
# ANA ÇALIŞTIRMA BLOĞU
# ==============================================================================
if __name__ == "__main__":
    CONFIG = {}
    config_path = os.path.join(APP_DIR, 'config.json')
    setup_logging(LOG_FILE_PATH) # Default log path ile başla
    try:
        with open(config_path, 'r', encoding='utf-8') as f: CONFIG = json.load(f)
        log_path_relative = CONFIG.get('SYSTEM', {}).get('LOG_FILE_RELATIVE', 'logs/sentinel.log')
        LOG_FILE_PATH = os.path.join(BASE_DIR, log_path_relative)
        setup_logging(LOG_FILE_PATH) # Final log path
        log_and_print(f"config.json başarıyla yüklendi: {config_path}")
    except FileNotFoundError: log_and_print(f"KRİTİK HATA: config.json bulunamadı! {config_path}", logging.CRITICAL); exit(1)
    except json.JSONDecodeError as e: log_and_print(f"KRİTİK HATA: config.json hatalı formatta! Hata: {e}", logging.CRITICAL); exit(1)
    except Exception as e: log_and_print(f"KRİTİK HATA: Başlangıç hatası: {e}\n{traceback.format_exc()}", logging.CRITICAL); exit(1)

    CONFIG.setdefault('SYSTEM', {}); CONFIG.setdefault('DEVICE_ID', f"DEFAULT_{socket.gethostname()}"); CONFIG.setdefault('MILESTONE', {'ENABLED': False})
    CONFIG.setdefault('FTP', {}); CONFIG.setdefault('SENSOR', {}); CONFIG.setdefault('PUSHOVER', {}); CONFIG.setdefault('EMAIL', {})

    sentinel = SentinelSystem(CONFIG)
    startup_message = f"SENTINEL GÜVENLİK SİSTEMİ v2.2 (Reboot Feature) - Cihaz ID: {CONFIG['DEVICE_ID']}"
    print(startup_message); log_and_print("Sistem başlatılıyor...")

    main_loop = asyncio.get_event_loop()
    try:
        main_task = main_loop.create_task(sentinel.run())
        main_loop.run_until_complete(main_task)
    except KeyboardInterrupt: log_and_print("Program kullanıcı tarafından sonlandırıldı (KeyboardInterrupt).")
    except SystemExit as e: log_and_print(f"Program SystemExit ile sonlandırıldı: {e}")
    except Exception as e: log_and_print(f"KRİTİK HATA: Beklenmeyen hata ana döngüden çıktı: {e}\n{traceback.format_exc()}", level=logging.CRITICAL)
    finally:
        log_and_print("Sistem kapatılıyor...")
        try:
             tasks = [t for t in asyncio.all_tasks(loop=main_loop) if t is not asyncio.current_task(loop=main_loop)]
             if tasks:
                 for task in tasks: task.cancel()
                 # İptallerin biraz işlemesi için bekle
                 # main_loop.run_until_complete(asyncio.sleep(1.0)) # Kapanışta hata verebilir
             if hasattr(sentinel, 'sensor') and sentinel.sensor: sentinel.sensor.cleanup()
             try: GPIO.cleanup(); log_and_print("GPIO başarıyla temizlendi.")
             except RuntimeError as e: # GPIO zaten temizlenmişse
                  if "cannot call cleanup()" in str(e): log_and_print("GPIO zaten temizlenmiş.", logging.DEBUG)
                  else: log_and_print(f"Kapanışta GPIO temizlenirken hata: {e}", logging.WARNING)
             except NameError: pass
             except Exception as e: log_and_print(f"Kapanışta GPIO temizlenirken hata: {e}", logging.WARNING)
        except Exception as e: log_and_print(f"Kapanış sırasında hata: {e}", logging.WARNING)
        print("\nSistem kapatıldı.");
        logging.shutdown()
