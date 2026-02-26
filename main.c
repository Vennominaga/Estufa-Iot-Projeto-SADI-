/*
 * ============================================================
 *   SISTEMA DE MONITORAMENTO E CONTROLE – ESTUFA AGRÍCOLA
 * ============================================================
 *   Microcontrolador : ESP32
 *   Sensor Temp/Umi  : DHT22
 *   Sensor Luz       : LDR (divisor de tensão → ADC)
 *   Atuadores        : Módulo Relé 2 canais – Active LOW
 *   Display          : LCD 16x2 com módulo I2C
 *   Conectividade    : WiFi Access Point + Servidor Web
 * ============================================================
 *
 *   MODO DE OPERAÇÃO WiFi
 *   ─────────────────────────────────────────
 *   O ESP32 cria sua própria rede WiFi (Access Point).
 *   Conecte seu dispositivo à rede criada pelo ESP32 e
 *   acesse o dashboard no navegador em: http://192.168.4.1
 *   
 *   Nome da rede padrão: "Estufa_ESP32"
 *   Senha padrão: "estufa123"
 *   (editáveis no início do código)
 *
 *   MAPEAMENTO DE PINOS (ESP32)
 *   ─────────────────────────────────────────
 *   DHT22  dados      → GPIO  4
 *   LDR    ADC        → GPIO 34  (ADC1_CH0)
 *   Relé Canal 1      → GPIO  2  (Lâmpada Grow)
 *   Relé Canal 2      → GPIO 15  (Motor / Ventilador)
 *   LCD I2C           → SDA GPIO 21 | SCL GPIO 22 (padrão ESP32)
 *
 *   CIRCUITO DO LDR (divisor de tensão)
 *   ─────────────────────────────────────────
 *         3V3
 *          │
 *         LDR          ← resistência varia com a luz
 *          │
 *     ┌────┴────┐
 *     │  GPIO 34│      ← pino ADC lê a tensão do ponto médio
 *     └────┬────┘
 *          │
 *        R 10kΩ        ← resistor fixo
 *          │
 *         GND
 *
 *   RELÉS – ATIVAÇÃO EM BORDA NEGATIVA (Active LOW)
 *   ─────────────────────────────────────────
 *     GPIO = LOW   → relé ENERGIZADO  (dispositivo LIGADO)
 *     GPIO = HIGH  → relé DE-ENERGIZADO (dispositivo DESLIGADO)
 *
 *   MODO AUTOMÁTICO – Lógica de controle com histérese
 *   ─────────────────────────────────────────
 *     Motor (ventilador):
 *       Liga   se temperatura > limiar  OU  umidade > limiar
 *       Desliga se temperatura < limiar  E   umidade < limiar
 *     Lâmpada Grow:
 *       Liga   se luminosidade < limiar (ambiente escuro)
 *       Desliga se luminosidade > limiar
 *     Todos os limiares são configuráveis pela interface web.
 *
 *   MODO MANUAL
 *   ─────────────────────────────────────────
 *     Cada relé pode ser ligado / desligado individualmente
 *     pela interface web; a lógica automática fica suspensa.
 *
 *   BIBLIOTECAS NECESSÁRIAS (Library Manager)
 *   ─────────────────────────────────────────
 *     • DHT sensor library        (Adafruit)
 *     • Adafruit Unified Sensor
 *     • LiquidCrystal_I2C         (Frank de Castelbajac)
 *     • WiFi.h / WebServer.h      (já incluídas no core ESP32)
 *
 * ============================================================
 */

#include <Arduino.h>
#include <DHT.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <WebServer.h>

// ──────────────────────────────────────────────────────────
//  CONFIGURAÇÃO DO ACCESS POINT  ← altere aqui antes de gravar
// ──────────────────────────────────────────────────────────
const char* AP_SSID  = "Estufa_ESP32";    // Nome da rede WiFi criada pelo ESP32
const char* AP_SENHA = "estufa123";       // Senha (mínimo 8 caracteres, deixe "" para rede aberta)

// ──────────────────────────────────────────────────────────
//  PINOS
// ──────────────────────────────────────────────────────────
#define PIN_DHT            4
#define PIN_LDR           34
#define PIN_RELAY_LAMPADA  2
#define PIN_RELAY_MOTOR   15

// ──────────────────────────────────────────────────────────
//  PARÂMETROS – valores padrão (editáveis em tempo real pela web)
// ──────────────────────────────────────────────────────────
#define TIPO_DHT  DHT22

// Histérese – Motor
float cfg_tempLigar   = 30.0;   // Motor liga   se T  >  este valor (°C)
float cfg_tempDeslig  = 27.0;   // Motor desliga se T  <  este valor (°C)
float cfg_umidLigar   = 70.0;   // Motor liga   se U  >  este valor (%)
float cfg_umidDeslig  = 60.0;   // Motor desliga se U  <  este valor (%)

// Histérese – Lâmpada
int   cfg_luzLigar    = 25;     // Lâmpada liga   se Luz <  este valor (%)
int   cfg_luzDeslig   = 35;     // Lâmpada desliga se Luz >  este valor (%)

// ──────────────────────────────────────────────────────────
//  TEMPORIZAÇÃO (ms)
// ──────────────────────────────────────────────────────────
#define TEMPO_TELA      10000   // 10 s entre telas no LCD
#define TEMPO_LEITURA    1000   // 1 s  entre leituras dos sensores

// ──────────────────────────────────────────────────────────
//  LCD
// ──────────────────────────────────────────────────────────
#define LCD_ENDERECO  0x27
#define LCD_COLUNAS     16
#define LCD_LINHAS       2

// ──────────────────────────────────────────────────────────
//  OBJETOS GLOBAIS
// ──────────────────────────────────────────────────────────
DHT               dht(PIN_DHT, TIPO_DHT);
LiquidCrystal_I2C lcd(LCD_ENDERECO, LCD_COLUNAS, LCD_LINHAS);
WebServer         server(80);

// ──────────────────────────────────────────────────────────
//  VARIÁVEIS DE ESTADO
// ──────────────────────────────────────────────────────────
float temperatura = 0.0;
float umidade     = 0.0;
int   pctLuz      = 0;

bool lampada      = false;     // estado real aplicado ao relé
bool motor        = false;

bool modoManual   = false;     // false = auto | true = manual
bool lampManual   = false;     // controle manual – lâmpada
bool motManual    = false;     // controle manual – motor

int           tela     = 0;    // 0 dados | 1 status | 2 rede
unsigned long tTroca   = 0;
unsigned long tLeitura = 0;

// ══════════════════════════════════════════════════════════
//  FUNÇÕES AUXILIARES
// ══════════════════════════════════════════════════════════
String pad16(const String& s) {
    String r = s;
    while ((int)r.length() < 16) r += ' ';
    return r.substring(0, 16);
}

// ══════════════════════════════════════════════════════════
//  WiFi – modo Access Point
// ══════════════════════════════════════════════════════════
void configurarAP() {
    WiFi.mode(WIFI_AP);
    
    bool apOk;
    if (strlen(AP_SENHA) >= 8) {
        // Access Point com senha (WPA2)
        apOk = WiFi.softAP(AP_SSID, AP_SENHA);
        Serial.println("Configurando Access Point com seguranca WPA2...");
    } else {
        // Access Point aberto (sem senha)
        apOk = WiFi.softAP(AP_SSID);
        Serial.println("Configurando Access Point ABERTO (sem senha)...");
    }
    
    if (apOk) {
        IPAddress ip = WiFi.softAPIP();
        Serial.println("Access Point ativo!");
        Serial.println("SSID: " + String(AP_SSID));
        Serial.println("IP: " + ip.toString());
        Serial.println("Conecte-se a rede '" + String(AP_SSID) + "' e acesse http://" + ip.toString());
    } else {
        Serial.println("Falha ao iniciar Access Point!");
    }
}

// ══════════════════════════════════════════════════════════
//  SENSORES
// ══════════════════════════════════════════════════════════
void lerSensores() {
    float t = dht.readTemperature();
    float h = dht.readHumidity();
    if (!isnan(t)) temperatura = t;
    if (!isnan(h)) umidade     = h;

    int adc = analogRead(PIN_LDR);
    pctLuz  = constrain(map(adc, 0, 4095, 0, 100), 0, 100);
}

// ══════════════════════════════════════════════════════════
//  ATUADORES  (lógica com histérese)
// ══════════════════════════════════════════════════════════
void controlar() {
    if (modoManual) {
        lampada = lampManual;
        motor   = motManual;
    } else {
        // Motor – liga por temperatura OU umidade (histérese dupla)
        if (!motor  && (temperatura > cfg_tempLigar  || umidade > cfg_umidLigar ))  motor  = true;
        if ( motor  && (temperatura < cfg_tempDeslig && umidade < cfg_umidDeslig))  motor  = false;

        // Lâmpada – liga quando ambiente escuro
        if (!lampada && pctLuz < cfg_luzLigar )  lampada = true;
        if ( lampada && pctLuz > cfg_luzDeslig)  lampada = false;
    }

    // Active LOW: LOW = ligado
    digitalWrite(PIN_RELAY_LAMPADA, lampada ? LOW : HIGH);
    digitalWrite(PIN_RELAY_MOTOR,   motor   ? LOW : HIGH);
}

// ══════════════════════════════════════════════════════════
//  LCD – três telas com rotação automática
// ══════════════════════════════════════════════════════════
void mostrarDados() {
    lcd.setCursor(0,0);
    lcd.print(pad16("T:" + String(temperatura,1) + "C  U:" + String((int)umidade) + "%"));
    lcd.setCursor(0,1);
    lcd.print(pad16("Luz: " + String(pctLuz) + "%"));
}

void mostrarStatus() {
    lcd.setCursor(0,0);
    lcd.print(pad16("Lampada: " + String(lampada ? "LIGADA " : "DESLIG.")));
    lcd.setCursor(0,1);
    lcd.print(pad16("Motor:   " + String(motor   ? "LIGADO " : "DESLIG.")));
}

void mostrarRede() {
    IPAddress ip = WiFi.softAPIP();
    String modo = modoManual ? "MANUAL" : "AUTO";
    lcd.setCursor(0,0); lcd.print(pad16("AP: " + ip.toString()));
    lcd.setCursor(0,1); lcd.print(pad16("Modo: " + modo));
}

void mostrarTela() {
    switch(tela) {
        case 0: mostrarDados();   break;
        case 1: mostrarStatus();  break;
        case 2: mostrarRede();    break;
    }
}

// ══════════════════════════════════════════════════════════
//  WEB SERVER – página HTML (dashboard completo)
// ══════════════════════════════════════════════════════════
void handleRoot() {
/* ─── HTML inicia aqui ─── */
static const char page[] =
R"ENDOFHTML(<!DOCTYPE html>
<html lang="pt-BR">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Dashboard Estufa</title>
<style>
  @import url('https://fonts.googleapis.com/css2?family=DM+Sans:wght@400;500;600&family=Orbitron:wght@600&display=swap');
  :root{
    --bg:#0d1117;--surface:#161b22;--border:#21262d;
    --green:#39d353;--green-dim:#1e7a2e;
    --amber:#e3a019;--amber-dim:#7a5510;
    --red:#f85149;--red-dim:#7a2010;
    --text:#c9d1d9;--text-dim:#8b949e;--text-bright:#f0f6fc;
    --radius:10px;
  }
  *{box-sizing:border-box;margin:0;padding:0}
  body{font-family:'DM Sans',sans-serif;background:var(--bg);color:var(--text);min-height:100vh;padding:20px 16px}

  .hdr{text-align:center;margin-bottom:22px}
  .hdr h1{font-family:'Orbitron',sans-serif;font-size:1.25rem;color:var(--text-bright);letter-spacing:2px;margin-bottom:6px}
  .hdr .meta{font-size:.75rem;color:var(--text-dim)}
  .hdr .meta .dot{display:inline-block;width:8px;height:8px;border-radius:50%;background:var(--green);margin-right:5px;animation:pulse 2s infinite}
  @keyframes pulse{0%,100%{opacity:1}50%{opacity:.3}}

  .mode-bar{display:flex;align-items:center;justify-content:center;gap:12px;margin-bottom:20px}
  .mode-badge{font-size:.78rem;font-weight:600;padding:5px 14px;border-radius:20px;letter-spacing:1px}
  .mode-badge.auto{background:var(--green-dim);color:var(--green)}
  .mode-badge.manual{background:var(--amber-dim);color:var(--amber)}
  .btn-mode{background:var(--surface);border:1px solid var(--border);color:var(--text);padding:5px 14px;border-radius:20px;font-size:.78rem;cursor:pointer;transition:.2s}
  .btn-mode:hover{border-color:var(--amber);color:var(--amber)}

  .cards{display:grid;grid-template-columns:repeat(3,1fr);gap:10px;max-width:680px;margin:0 auto 20px}
  .card{background:var(--surface);border:1px solid var(--border);border-radius:var(--radius);padding:16px 10px;text-align:center;position:relative;overflow:hidden;transition:border-color .3s}
  .card::before{content:'';position:absolute;inset:0;opacity:.06;background:linear-gradient(135deg,var(--accent),transparent)}
  .card[data-color=green]{--accent:var(--green);border-color:var(--green-dim)}
  .card[data-color=amber]{--accent:var(--amber);border-color:var(--amber-dim)}
  .card[data-color=red]  {--accent:var(--red);  border-color:var(--red-dim)}
  .card .ico{font-size:1.5rem;position:relative}
  .card .lbl{font-size:.68rem;text-transform:uppercase;letter-spacing:1.5px;color:var(--text-dim);margin:5px 0 3px;position:relative}
  .card .val{font-size:1.6rem;font-weight:600;color:var(--text-bright);position:relative}
  .card .unit{font-size:.68rem;color:var(--text-dim);position:relative}
  .card .bar-bg{margin-top:10px;height:4px;background:var(--border);border-radius:2px;overflow:hidden;position:relative}
  .card .bar-fill{height:100%;border-radius:2px;background:var(--green);width:0%;transition:width .6s ease,background .4s}
  .card .bar-fill.amber{background:var(--amber)}
  .card .bar-fill.red   {background:var(--red)}

  .sec{max-width:680px;margin:0 auto 20px}
  .sec-t{font-size:.68rem;text-transform:uppercase;letter-spacing:2px;color:var(--text-dim);border-bottom:1px solid var(--border);padding-bottom:6px;margin-bottom:10px}
  .relay{display:flex;align-items:center;justify-content:space-between;background:var(--surface);border:1px solid var(--border);border-radius:var(--radius);padding:14px 16px;margin-bottom:8px;transition:border-color .3s}
  .relay.on{border-color:var(--green-dim)}
  .relay .rname{font-size:.88rem;font-weight:500}
  .relay .ractions{display:flex;align-items:center;gap:10px}
  .badge{font-size:.72rem;font-weight:600;padding:3px 10px;border-radius:12px;letter-spacing:.5px}
  .badge.on {background:var(--green-dim);color:var(--green)}
  .badge.off{background:var(--border);color:var(--text-dim)}
  .btn-relay{background:var(--surface);border:1px solid var(--border);color:var(--text);padding:4px 12px;border-radius:6px;font-size:.76rem;cursor:pointer;transition:.2s;display:none}
  .btn-relay:hover{opacity:.8}
  .btn-relay.ligar   {border-color:var(--green);color:var(--green)}
  .btn-relay.desligar{border-color:var(--red);  color:var(--red)}

  .cfg-grid{display:grid;grid-template-columns:1fr 1fr;gap:8px}
  .cfg-item{background:var(--surface);border:1px solid var(--border);border-radius:var(--radius);padding:10px}
  .cfg-item label{font-size:.7rem;color:var(--text-dim);display:block;margin-bottom:4px}
  .cfg-item input{width:100%;padding:5px 8px;border-radius:6px;border:1px solid var(--border);background:var(--bg);color:var(--text-bright);font-size:.85rem;outline:none;transition:border-color .2s}
  .cfg-item input:focus{border-color:var(--amber)}
  .cfg-group-lbl{grid-column:1/-1;font-size:.72rem;color:var(--amber);font-weight:600;margin-top:4px;padding-top:4px;border-top:1px solid var(--border)}
  .btn-save{width:100%;margin-top:14px;padding:9px;background:var(--green-dim);border:1px solid var(--green);color:var(--green);border-radius:var(--radius);font-size:.82rem;font-weight:600;cursor:pointer;transition:.2s}
  .btn-save:hover{background:var(--green);color:#0d1117}

  .toast{position:fixed;bottom:24px;left:50%;transform:translateX(-50%) translateY(60px);background:var(--surface);border:1px solid var(--green);color:var(--green);padding:9px 22px;border-radius:20px;font-size:.8rem;font-weight:600;transition:transform .3s;z-index:99;pointer-events:none}
  .toast.show{transform:translateX(-50%) translateY(0)}

  @media(max-width:480px){
    .cards{grid-template-columns:1fr 1fr}
    .cfg-grid{grid-template-columns:1fr}
  }
</style>
</head>
<body>

<div class="hdr">
  <h1>🌿 ESTUFA</h1>
  <div class="meta"><span class="dot"></span>Conectado &nbsp;|&nbsp; IP: <span id="ipAddr">–</span> &nbsp;|&nbsp; <span id="upd">–</span></div>
</div>

<div class="mode-bar">
  <span class="mode-badge auto" id="modeBadge">AUTO</span>
  <button class="btn-mode" id="modeBtn" onclick="toggleMode()">Ativar Manual</button>
</div>

<div class="cards">
  <div class="card" id="cardTemp" data-color="green">
    <div class="ico">🌡️</div>
    <div class="lbl">Temperatura</div>
    <div class="val" id="vTemp">–</div>
    <div class="unit">°C</div>
    <div class="bar-bg"><div class="bar-fill" id="barTemp"></div></div>
  </div>
  <div class="card" id="cardUmid" data-color="green">
    <div class="ico">💧</div>
    <div class="lbl">Umidade</div>
    <div class="val" id="vUmid">–</div>
    <div class="unit">%</div>
    <div class="bar-bg"><div class="bar-fill" id="barUmid"></div></div>
  </div>
  <div class="card" id="cardLuz" data-color="green">
    <div class="ico">☀️</div>
    <div class="lbl">Luminosidade</div>
    <div class="val" id="vLuz">–</div>
    <div class="unit">%</div>
    <div class="bar-bg"><div class="bar-fill" id="barLuz"></div></div>
  </div>
</div>

<div class="sec">
  <div class="sec-t">Relés</div>
  <div class="relay" id="rowLamp">
    <span class="rname">💡 Lâmpada Grow</span>
    <div class="ractions">
      <span class="badge off" id="bLamp">DESLIG.</span>
      <button class="btn-relay" id="btnLamp"></button>
    </div>
  </div>
  <div class="relay" id="rowMot">
    <span class="rname">🌀 Motor / Ventilador</span>
    <div class="ractions">
      <span class="badge off" id="bMot">DESLIG.</span>
      <button class="btn-relay" id="btnMot"></button>
    </div>
  </div>
</div>

<div class="sec">
  <div class="sec-t">⚙️ Limiares – modo automático</div>
  <div class="cfg-grid">
    <div class="cfg-group-lbl">🌀 Motor / Ventilador</div>
    <div class="cfg-item"><label>Temperatura ligar (°C)</label><input type="number" id="cTL" step="0.5"></div>
    <div class="cfg-item"><label>Temperatura desligar (°C)</label><input type="number" id="cTD" step="0.5"></div>
    <div class="cfg-item"><label>Umidade ligar (%)</label><input type="number" id="cUL" step="1"></div>
    <div class="cfg-item"><label>Umidade desligar (%)</label><input type="number" id="cUD" step="1"></div>
    <div class="cfg-group-lbl">💡 Lâmpada Grow</div>
    <div class="cfg-item"><label>Luminosidade ligar (%)</label><input type="number" id="cLL" step="1"></div>
    <div class="cfg-item"><label>Luminosidade desligar (%)</label><input type="number" id="cLD" step="1"></div>
  </div>
  <button class="btn-save" onclick="saveConfig()">💾 Salvar configurações</button>
</div>

<div class="toast" id="toast">Salvo</div>

<script>
let isManual=false;

async function poll(){
  try{
    const r=await fetch('/api/data');
    const d=await r.json();
    document.getElementById('vTemp').textContent=d.temp;
    document.getElementById('vUmid').textContent=d.umid;
    document.getElementById('vLuz').textContent=d.luz;
    updateBar('barTemp','cardTemp',d.temp,27,30);
    updateBar('barUmid','cardUmid',d.umid,60,70);
    updateBar('barLuz','cardLuz',d.luz,35,75);
    updRelay('rowLamp','bLamp','btnLamp',d.lampada,'lamp');
    updRelay('rowMot','bMot','btnMot',d.motor,'motor');
    isManual=d.modoManual===1;
    updateModeUI();
    setIfBlur('cTL',d.tempLigar);
    setIfBlur('cTD',d.tempDeslig);
    setIfBlur('cUL',d.umidLigar);
    setIfBlur('cUD',d.umidDeslig);
    setIfBlur('cLL',d.luzLigar);
    setIfBlur('cLD',d.luzDeslig);
    document.getElementById('upd').textContent=new Date().toLocaleTimeString();
  }catch(e){}
}
setInterval(poll,1200);

function setIfBlur(id,v){
  const el=document.getElementById(id);
  if(document.activeElement!==el) el.value=v;
}

function updateBar(barId,cardId,val,warn,danger){
  document.getElementById(barId).style.width=Math.min(val,100)+'%';
  document.getElementById(barId).className='bar-fill'+(val>=danger?' red':val>=warn?' amber':'');
  document.getElementById(cardId).dataset.color=val>=danger?'red':val>=warn?'amber':'green';
}

function updRelay(rowId,badgeId,btnId,state,ch){
  const on=state===1;
  const row=document.getElementById(rowId);
  const badge=document.getElementById(badgeId);
  const btn=document.getElementById(btnId);
  row.className='relay'+(on?' on':'');
  badge.textContent=on?'LIGADO':'DESLIG.';
  badge.className='badge '+(on?'on':'off');
  if(isManual){
    btn.style.display='inline-block';
    btn.textContent=on?'Desligar':'Ligar';
    btn.className='btn-relay '+(on?'desligar':'ligar');
    btn.onclick=()=>setRelay(ch,on?0:1);
  } else btn.style.display='none';
}

function updateModeUI(){
  const b=document.getElementById('modeBadge');
  const btn=document.getElementById('modeBtn');
  if(isManual){b.textContent='MANUAL';b.className='mode-badge manual';btn.textContent='Ativar Auto';}
  else        {b.textContent='AUTO';  b.className='mode-badge auto';  btn.textContent='Ativar Manual';}
}

async function toggleMode(){ await post('mode',{mode:isManual?0:1}); }
async function setRelay(ch,st){ await post('relay',{channel:ch,state:st}); }

async function saveConfig(){
  const tl=+document.getElementById('cTL').value;
  const td=+document.getElementById('cTD').value;
  if(td>=tl){showToast('Temp desligar deve ser menor que ligar',true);return;}
  const ul=+document.getElementById('cUL').value;
  const ud=+document.getElementById('cUD').value;
  if(ud>=ul){showToast('Umid desligar deve ser menor que ligar',true);return;}
  const ll=+document.getElementById('cLL').value;
  const ld=+document.getElementById('cLD').value;
  if(ld<=ll){showToast('Luz desligar deve ser maior que ligar',true);return;}
  await post('config',{tempLigar:tl,tempDeslig:td,umidLigar:ul,umidDeslig:ud,luzLigar:ll,luzDeslig:ld});
  showToast('Configurações salvas ✓');
}

async function post(endpoint,data){
  try{
    const r=await fetch('/api/'+endpoint,{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:new URLSearchParams(data).toString()});
    return await r.json();
  }catch(e){showToast('Erro de comunicação',true);}
}

function showToast(msg,isErr){
  const t=document.getElementById('toast');
  t.textContent=msg;
  t.style.borderColor=isErr?'var(--red)':'var(--green)';
  t.style.color=isErr?'var(--red)':'var(--green)';
  t.classList.add('show');
  setTimeout(()=>t.classList.remove('show'),2200);
}

document.getElementById('ipAddr').textContent=window.location.hostname;
</script>
</body>
</html>
)ENDOFHTML";
/* ─── HTML termina aqui ─── */

    server.sendHeader("Connection","close");
    server.send(200, "text/html; charset=UTF-8", page);
}

// ══════════════════════════════════════════════════════════
//  WEB SERVER – API endpoints
// ══════════════════════════════════════════════════════════

/** GET /api/data – retorna JSON com leituras e configuração atual */
void handleGetData() {
    String j;
    j.reserve(256);
    j  = "{\"temp\":"    + String(temperatura, 1);
    j += ",\"umid\":"    + String(umidade, 1);
    j += ",\"luz\":"     + String(pctLuz);
    j += ",\"lampada\":" + String(lampada ? 1 : 0);
    j += ",\"motor\":"   + String(motor   ? 1 : 0);
    j += ",\"modoManual\":" + String(modoManual ? 1 : 0);
    j += ",\"tempLigar\":"  + String(cfg_tempLigar,  1);
    j += ",\"tempDeslig\":" + String(cfg_tempDeslig, 1);
    j += ",\"umidLigar\":"  + String(cfg_umidLigar,  1);
    j += ",\"umidDeslig\":" + String(cfg_umidDeslig, 1);
    j += ",\"luzLigar\":"   + String(cfg_luzLigar);
    j += ",\"luzDeslig\":"  + String(cfg_luzDeslig);
    j += "}";
    server.sendHeader("Cache-Control","no-store");
    server.send(200, "application/json", j);
}

/** POST /api/mode – alterna modo automático / manual */
void handleSetMode() {
    if (!server.hasArg("mode")) { server.send(400,"text/plain","falta 'mode'"); return; }

    bool novoManual = (server.arg("mode").toInt() == 1);

    if (novoManual && !modoManual) {
        // entrando no modo manual: captura estados atuais dos relés
        lampManual = lampada;
        motManual  = motor;
    }
    modoManual = novoManual;

    server.send(200,"application/json","{\"ok\":1}");
    Serial.println("[WEB] modo -> " + String(modoManual ? "MANUAL" : "AUTO"));
}

/** POST /api/relay – liga/desliga um relé no modo manual */
void handleSetRelay() {
    if (!server.hasArg("channel") || !server.hasArg("state")) {
        server.send(400,"text/plain","falta 'channel' ou 'state'"); return;
    }
    if (!modoManual) {
        server.send(403,"text/plain","modo automatico ativo"); return;
    }

    String ch = server.arg("channel");
    int    st = server.arg("state").toInt();

    if (ch == "lamp")  lampManual = (st == 1);
    if (ch == "motor") motManual  = (st == 1);

    controlar();   // aplica imediatamente aos relés
    server.send(200,"application/json","{\"ok\":1}");
    Serial.println("[WEB] relay " + ch + " -> " + String(st));
}

/** POST /api/config – atualiza limiares de controle automático */
void handleSetConfig() {
    if (server.hasArg("tempLigar"))  cfg_tempLigar  = server.arg("tempLigar").toFloat();
    if (server.hasArg("tempDeslig")) cfg_tempDeslig = server.arg("tempDeslig").toFloat();
    if (server.hasArg("umidLigar"))  cfg_umidLigar  = server.arg("umidLigar").toFloat();
    if (server.hasArg("umidDeslig")) cfg_umidDeslig = server.arg("umidDeslig").toFloat();
    if (server.hasArg("luzLigar"))   cfg_luzLigar   = server.arg("luzLigar").toInt();
    if (server.hasArg("luzDeslig"))  cfg_luzDeslig  = server.arg("luzDeslig").toInt();

    server.send(200,"application/json","{\"ok\":1}");
    Serial.println("[WEB] config atualizada");
}

void handleNotFound() { server.send(404,"text/plain","Not Found"); }

// ══════════════════════════════════════════════════════════
//  Registra as rotas do servidor
// ══════════════════════════════════════════════════════════
void iniciarWebServer() {
    server.on("/",            HTTP_GET,  handleRoot);
    server.on("/api/data",    HTTP_GET,  handleGetData);
    server.on("/api/mode",    HTTP_POST, handleSetMode);
    server.on("/api/relay",   HTTP_POST, handleSetRelay);
    server.on("/api/config",  HTTP_POST, handleSetConfig);
    server.onNotFound(handleNotFound);
    server.begin();
    Serial.println("Servidor web iniciado – acesse http://" + WiFi.softAPIP().toString());
}

// ══════════════════════════════════════════════════════════
//  SETUP
// ══════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);
    Serial.println("\n=== Sistema Estufa Iniciando ===");

    // ── relés: HIGH = desligado (active LOW) ──
    pinMode(PIN_RELAY_LAMPADA, OUTPUT);
    pinMode(PIN_RELAY_MOTOR,   OUTPUT);
    digitalWrite(PIN_RELAY_LAMPADA, HIGH);
    digitalWrite(PIN_RELAY_MOTOR,   HIGH);

    // ── LCD boot ──
    lcd.init();
    lcd.backlight();
    lcd.clear();
    lcd.setCursor(0,0); lcd.print(pad16("Sistema Estufa"));
    lcd.setCursor(0,1); lcd.print(pad16("Iniciando AP..."));

    // ── WiFi Access Point ──
    configurarAP();

    // Mostra IP no LCD por 3 s
    lcd.clear();
    lcd.setCursor(0,0); lcd.print(pad16("AP: " + String(AP_SSID)));
    lcd.setCursor(0,1); lcd.print(pad16(WiFi.softAPIP().toString()));
    delay(3000);
    lcd.clear();

    // ── Web Server ──
    iniciarWebServer();

    // ── DHT22 ──
    dht.begin();
    delay(2000);   // estabilização após power-on

    // ── primeira leitura ──
    lerSensores();
    controlar();
    mostrarTela();

    tTroca   = millis();
    tLeitura = millis();

    Serial.println("Setup concluido.\n");
}

// ══════════════════════════════════════════════════════════
//  LOOP PRINCIPAL
// ══════════════════════════════════════════════════════════
void loop() {
    // ── atende requisições HTTP ──
    server.handleClient();

    unsigned long agora = millis();

    // ── leitura periódica dos sensores (1 s) ──
    if (agora - tLeitura >= TEMPO_LEITURA) {
        tLeitura = agora;

        lerSensores();
        controlar();
        mostrarTela();

        // debug no Monitor Serie
        Serial.print("T:");    Serial.print(temperatura, 1);
        Serial.print(" U:");   Serial.print((int)umidade);
        Serial.print(" Luz:");  Serial.print(pctLuz);
        Serial.print(" Lamp:"); Serial.print(lampada ? "ON" : "OFF");
        Serial.print(" Mot:");  Serial.print(motor   ? "ON" : "OFF");
        Serial.print(" Modo:"); Serial.println(modoManual ? "MAN" : "AUTO");
    }

    // ── rotação de telas no LCD (3 telas, 10 s cada) ──
    if (agora - tTroca >= TEMPO_TELA) {
        tTroca = agora;
        tela   = (tela + 1) % 3;
        mostrarTela();
    }
}
