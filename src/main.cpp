#include <M5StickCPlus2.h>
#include <BluetoothSerial.h>

// BPM計算用の変数
#define SAMPLE_RATE 50                // サンプリングレート（Hz）
#define BPM_BUFFER_SIZE 300           // 約6秒分のデータ（50Hz×6秒）
#define STEP_THRESHOLD 0.15            // 歩行検出の閾値を下げる（0.8から0.15に）
#define MIN_STEP_INTERVAL 500          // 最小ステップ間隔を長くする（300msから500msに）

// Bluetooth設定
BluetoothSerial SerialBT;
bool btConnected = false;

// IMU 変数
float accX = 0.0F;
float accY = 0.0F;
float accZ = 0.0F;
bool imuInitialized = false;

// グローバル変数
float accBuffer[BPM_BUFFER_SIZE][3];  // XYZ加速度バッファ
int bufferIndex = 0;                  // バッファのインデックス
unsigned long lastStepTime = 0;       // 最後のステップ検出時間
unsigned long stepTimes[20];          // 直近20回の歩行タイミング
int stepIndex = 0;                    // ステップ配列のインデックス
float currentBPM = 0.0;               // 現在のBPM

// 画面表示設定
#define SCREEN_WIDTH 135
#define SCREEN_HEIGHT 240
#define GRAPH_HEIGHT 100
#define GRAPH_Y_POS 60

// データ送信用
#define DATA_INTERVAL 100             // データ送信間隔（ms）
unsigned long lastDataTime = 0;       // 最後のデータ送信時間

// デバッグ用
#define DEBUG_INTERVAL 500            // デバッグ出力間隔（ms）
unsigned long lastDebugTime = 0;      // 最後のデバッグ出力時間

// 解析モード設定
bool sendRawData = false;             // 生データ送信モード（trueなら加速度の生データを送信）

// 関数宣言
void updateBPM();
void drawGraph();
void updateDisplay();
void handleButton();
void sendData();
void printDebugInfo();
void btCallback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param);
void displayStatusLine(const char* text, int line = 0, uint16_t color = WHITE);

void setup() {
  // シリアル通信初期化（最初に初期化）
  Serial.begin(115200);
  delay(100);
  Serial.println("\n\n\n*** Starting M5 Gait Analysis ***");
  Serial.println("-------------------------------");
  Serial.println("Serial Initialized.");
  
  // M5StickCPlus2の初期化
  Serial.println("Configuring M5StickCPlus2...");
  auto cfg = M5.config();  // 正しい初期化方法に戻す
  cfg.internal_imu = true;     // 内蔵IMUを有効化
  cfg.clear_display = true;    // 起動時に画面をクリア
  Serial.println("Starting StickCP2.begin()...");
  StickCP2.begin(cfg);
  Serial.println("StickCP2.begin() completed.");
  
  // 画面向きを設定（縦向き、USBケーブルが下）
  Serial.println("Setting up display...");
  StickCP2.Lcd.setRotation(1);
  StickCP2.Lcd.fillScreen(BLACK);
  StickCP2.Lcd.setTextSize(1);
  StickCP2.Lcd.setCursor(0, 0);
  StickCP2.Lcd.println("Initializing...");
  Serial.println("Display initialized.");
  
  // IMUの初期化状態を確認
  Serial.println("Checking IMU...");
  imuInitialized = StickCP2.Imu.isEnabled();
  if (imuInitialized) {
    Serial.println("IMU initialized successfully");
    displayStatusLine("IMU OK", GREEN);
    
    // 初期値を読み取って確認
    StickCP2.Imu.getAccelData(&accX, &accY, &accZ);
    Serial.printf("Initial accel: X:%6.2f, Y:%6.2f, Z:%6.2f\n", accX, accY, accZ);
  } else {
    Serial.println("IMU initialization failed");
    displayStatusLine("IMU failed!", RED);
  }
  Serial.println("IMU Check Done.");
  
  // Bluetooth初期化
  Serial.println("Initializing Bluetooth...");
  SerialBT.begin("M5StickCPlus2-Gait");
  SerialBT.register_callback(btCallback);
  Serial.println("Bluetooth Initialized.");
  
  // その他の初期化処理
  Serial.println("Initializing other variables...");
  lastStepTime = millis();
  stepIndex = 0;
  currentBPM = 0.0;
  Serial.println("Variable initialization complete.");
  
  // 初期化完了
  Serial.println("Setup Complete. Entering Loop...");
  displayStatusLine("Ready!", GREEN);
  delay(1000);
  StickCP2.Lcd.fillScreen(BLACK);
  
  // メイン画面の表示
  StickCP2.Lcd.fillScreen(BLACK);
  StickCP2.Lcd.setTextSize(2);
  StickCP2.Lcd.setCursor(0, 0);
  StickCP2.Lcd.println("Gait Analysis");
  StickCP2.Lcd.setCursor(0, 20);
  StickCP2.Lcd.println("Analyzing...");
}

// 画面に状態メッセージを表示する関数
void displayStatusLine(const char* text, int line, uint16_t color) {
  // 1行の高さは8ピクセル（フォントサイズ1の場合）
  int y = line * 10;
  
  // 表示する行のみ消去
  StickCP2.Lcd.fillRect(0, y, SCREEN_WIDTH, 10, BLACK);
  
  StickCP2.Lcd.setTextColor(color);
  StickCP2.Lcd.setCursor(0, y);
  StickCP2.Lcd.println(text);
  StickCP2.Lcd.setTextColor(WHITE); // デフォルトに戻す
}

void loop() {
  // M5StickCPlusのボタン状態を更新
  M5.update();

  // ボタン状態更新 - シリアルコマンドに置き換えた簡易版のみ使用
  handleButton();

  // 現在の時間を取得
  unsigned long currentTime = millis();

  // 加速度データ取得
  if (imuInitialized) {
    // BMI270から加速度データを取得
    if (StickCP2.Imu.update()) {  // 新しいデータが利用可能か確認
      StickCP2.Imu.getAccelData(&accX, &accY, &accZ);
      
      // デバッグ表示
      if (currentTime - lastDebugTime > DEBUG_INTERVAL) {
        Serial.printf("Accel: X:%6.2f, Y:%6.2f, Z:%6.2f\n", accX, accY, accZ);
        lastDebugTime = currentTime;
      }
      
      // 加速度の正規化（重力加速度を1.0gとして）
      float magnitude = sqrt(accX*accX + accY*accY + accZ*accZ);
      if (magnitude > 0) {
        accX /= magnitude;
        accY /= magnitude;
        accZ /= magnitude;
      }
      
      // ローパスフィルターを適用（ノイズ除去）
      static float lastX = 0, lastY = 0, lastZ = 0;
      const float alpha = 0.1;  // フィルター係数（0.1 = 強い平滑化）
      accX = alpha * accX + (1 - alpha) * lastX;
      accY = alpha * accY + (1 - alpha) * lastY;
      accZ = alpha * accZ + (1 - alpha) * lastZ;
      lastX = accX;
      lastY = accY;
      lastZ = accZ;
      
      // バッファに加速度データを保存
      accBuffer[bufferIndex][0] = accX;
      accBuffer[bufferIndex][1] = accY;
      accBuffer[bufferIndex][2] = accZ;
      bufferIndex = (bufferIndex + 1) % BPM_BUFFER_SIZE;
      
      // 歩行ステップ検出（Z軸の加速度変化を監視）
      float accMagnitude = sqrt(accX*accX + accY*accY + accZ*accZ);
      static float lastMagnitude = 1.0;  // 重力分
      float delta = abs(accMagnitude - lastMagnitude);
      
      // 閾値を超える変化があり、かつ最小間隔以上経過している場合にステップとして検出
      if (delta > STEP_THRESHOLD && (currentTime - lastStepTime) > MIN_STEP_INTERVAL) {
        lastStepTime = currentTime;
        
        // ステップ検出をシリアル出力
        Serial.print("Step detected! Delta: ");
        Serial.print(delta);
        Serial.print(", Magnitude: ");
        Serial.println(accMagnitude);
        
        // ステップ時間を記録
        stepTimes[stepIndex] = currentTime;
        stepIndex = (stepIndex + 1) % 20;
        
        // BPM更新
        updateBPM();
      }
      
      lastMagnitude = accMagnitude;
      
      // 画面更新（100ms毎）
      static unsigned long lastDisplayTime = 0;
      if (currentTime - lastDisplayTime > 100) {
        // 詳細な画面表示に更新
        updateDisplay();
        lastDisplayTime = currentTime;
      }
      
      // Bluetoothデータ送信
      if (currentTime - lastDataTime > DATA_INTERVAL && btConnected) {
        sendData();
        lastDataTime = currentTime;
      }
    }
  } else {
    // IMUが初期化されていない場合の処理
    Serial.println("IMU not initialized");
    delay(1000);
  }
  
  // サンプリングレート維持
  delay(5);  // 最大200Hzに制限（実際はIMUの更新レートに依存）
}

// デバッグ情報出力関数
void printDebugInfo() {
  Serial.println("--- DEBUG INFO ---");
  Serial.print("Accelerometer (x,y,z): ");
  Serial.print(accX);
  Serial.print(", ");
  Serial.print(accY);
  Serial.print(", ");
  Serial.println(accZ);
  
  float accMagnitude = sqrt(accX*accX + accY*accY + accZ*accZ);
  Serial.print("Acc Magnitude: ");
  Serial.print(accMagnitude);
  Serial.print(", Current BPM: ");
  Serial.println(currentBPM);
  
  Serial.print("Step Threshold: ");
  Serial.print(STEP_THRESHOLD);
  Serial.print(", BT Connected: ");
  Serial.println(btConnected ? "YES" : "NO");
  
  // ステップ間隔情報
  if (stepIndex > 0) {
    int prev = (stepIndex - 1 + 20) % 20;
    int prevprev = (stepIndex - 2 + 20) % 20;
    if (stepTimes[prev] > 0 && stepTimes[prevprev] > 0) {
      unsigned long interval = stepTimes[prev] - stepTimes[prevprev];
      Serial.print("Last Step Interval: ");
      Serial.print(interval);
      Serial.print("ms, Last Step Time: ");
      Serial.println(millis() - stepTimes[prev]);
    }
  }
  Serial.println("-----------------");
  Serial.flush(); // シリアルバッファをフラッシュして確実に出力する
}

// BPM計算関数
void updateBPM() {
  Serial.println("updateBPM Start");
  // 直近のステップ時間から平均BPMを計算
  int validSteps = 0;
  unsigned long totalInterval = 0;
  unsigned long currentTime = millis();
  
  // 最低5回のステップがあれば計算
  for (int i = 0; i < 19; i++) {
    if (stepTimes[i] > 0 && stepTimes[i+1] > 0 && stepTimes[i+1] > stepTimes[i]) {
      totalInterval += (stepTimes[i+1] - stepTimes[i]);
      validSteps++;
    }
  }
  
  if (validSteps >= 5) {
    float avgInterval = totalInterval / (float)validSteps;
    Serial.print("Calculated avgInterval: ");
    Serial.println(avgInterval);
    
    // ゼロ除算チェック
    if (avgInterval > 0) {
      float newBPM = 60000.0 / avgInterval;
      
      // 異常値をフィルタリング（40-200 BPMの範囲内に制限）
      if (newBPM < 40.0) newBPM = 40.0;
      if (newBPM > 200.0) newBPM = 200.0;
      
      // BPM更新のログ
      Serial.print("BPM updated: ");
      Serial.print(currentBPM);
      Serial.print(" -> ");
      Serial.print(newBPM);
      Serial.print(" (Avg Interval: ");
      Serial.print(avgInterval);
      Serial.print("ms, Valid Steps: ");
      Serial.print(validSteps);
      Serial.println(")");
      
      currentBPM = newBPM;
    } else {
      Serial.println("Error: avgInterval is zero or negative. Skipping BPM calculation.");
    }
  }
  Serial.println("updateBPM End");
}

// 画面表示更新
void updateDisplay() {
  // 上部情報表示領域
  StickCP2.Lcd.fillRect(0, 0, SCREEN_WIDTH, 60, BLACK);
  StickCP2.Lcd.setCursor(0, 0);
  StickCP2.Lcd.setTextSize(2);
  StickCP2.Lcd.println("Gait Analysis");
  
  StickCP2.Lcd.setCursor(0, 25);
  StickCP2.Lcd.print("BPM: ");
  StickCP2.Lcd.print(currentBPM, 1);
  
  // 動作モード表示
  StickCP2.Lcd.setCursor(0, 45);
  StickCP2.Lcd.print("Mode: ");
  if (sendRawData) {
    StickCP2.Lcd.print("RAW");
  } else {
    StickCP2.Lcd.print("BPM");
  }
  
  // グラフ表示
  drawGraph();
  
  // 下部情報表示
  StickCP2.Lcd.fillRect(0, GRAPH_Y_POS + GRAPH_HEIGHT + 5, SCREEN_WIDTH, 30, BLACK);
  StickCP2.Lcd.setCursor(0, GRAPH_Y_POS + GRAPH_HEIGHT + 10);
  StickCP2.Lcd.setTextSize(1);
  
  // BT接続状態表示
  StickCP2.Lcd.print("BT: ");
  StickCP2.Lcd.print(btConnected ? "Connected" : "Waiting");
  
  // IMU状態表示
  StickCP2.Lcd.setCursor(SCREEN_WIDTH-40, GRAPH_Y_POS + GRAPH_HEIGHT + 10);
  StickCP2.Lcd.print("IMU:");
  StickCP2.Lcd.print(imuInitialized ? "OK" : "NG");
  
  // ステップ間隔表示
  if (stepTimes[stepIndex > 0 ? stepIndex - 1 : 19] > 0 && 
      stepTimes[stepIndex > 1 ? stepIndex - 2 : 18] > 0) {
    unsigned long lastInterval = stepTimes[stepIndex > 0 ? stepIndex - 1 : 19] - 
                                stepTimes[stepIndex > 1 ? stepIndex - 2 : 18];
    StickCP2.Lcd.setCursor(0, GRAPH_Y_POS + GRAPH_HEIGHT + 25);
    StickCP2.Lcd.print("Interval: ");
    StickCP2.Lcd.print(lastInterval);
    StickCP2.Lcd.print("ms");
  }
}

// グラフ描画関数
void drawGraph() {
  // グラフ背景
  StickCP2.Lcd.fillRect(0, GRAPH_Y_POS, SCREEN_WIDTH, GRAPH_HEIGHT, DARKGREY);
  
  // X軸（時間）、Y軸（加速度）のグリッド線
  StickCP2.Lcd.drawLine(0, GRAPH_Y_POS + GRAPH_HEIGHT/2, SCREEN_WIDTH, GRAPH_Y_POS + GRAPH_HEIGHT/2, LIGHTGREY);
  
  // 加速度波形プロット（Z軸）
  for (int i = 0; i < SCREEN_WIDTH - 1; i++) {
    int idx1 = (bufferIndex - SCREEN_WIDTH + i + BPM_BUFFER_SIZE) % BPM_BUFFER_SIZE;
    int idx2 = (bufferIndex - SCREEN_WIDTH + i + 1 + BPM_BUFFER_SIZE) % BPM_BUFFER_SIZE;
    
    int y1 = GRAPH_Y_POS + GRAPH_HEIGHT/2 - (int)(accBuffer[idx1][2] * GRAPH_HEIGHT/4);
    int y2 = GRAPH_Y_POS + GRAPH_HEIGHT/2 - (int)(accBuffer[idx2][2] * GRAPH_HEIGHT/4);
    
    // 値の範囲チェック
    if (y1 < GRAPH_Y_POS) y1 = GRAPH_Y_POS;
    if (y1 > GRAPH_Y_POS + GRAPH_HEIGHT) y1 = GRAPH_Y_POS + GRAPH_HEIGHT;
    if (y2 < GRAPH_Y_POS) y2 = GRAPH_Y_POS;
    if (y2 > GRAPH_Y_POS + GRAPH_HEIGHT) y2 = GRAPH_Y_POS + GRAPH_HEIGHT;
    
    StickCP2.Lcd.drawLine(i, y1, i+1, y2, GREEN);
  }
  
  // X軸（赤）、Y軸（青）の加速度も表示
  for (int i = 0; i < SCREEN_WIDTH - 1; i++) {
    int idx1 = (bufferIndex - SCREEN_WIDTH + i + BPM_BUFFER_SIZE) % BPM_BUFFER_SIZE;
    int idx2 = (bufferIndex - SCREEN_WIDTH + i + 1 + BPM_BUFFER_SIZE) % BPM_BUFFER_SIZE;
    
    // X軸（赤）
    int y1 = GRAPH_Y_POS + GRAPH_HEIGHT/2 - (int)(accBuffer[idx1][0] * GRAPH_HEIGHT/4);
    int y2 = GRAPH_Y_POS + GRAPH_HEIGHT/2 - (int)(accBuffer[idx2][0] * GRAPH_HEIGHT/4);
    
    if (y1 < GRAPH_Y_POS) y1 = GRAPH_Y_POS;
    if (y1 > GRAPH_Y_POS + GRAPH_HEIGHT) y1 = GRAPH_Y_POS + GRAPH_HEIGHT;
    if (y2 < GRAPH_Y_POS) y2 = GRAPH_Y_POS;
    if (y2 > GRAPH_Y_POS + GRAPH_HEIGHT) y2 = GRAPH_Y_POS + GRAPH_HEIGHT;
    
    StickCP2.Lcd.drawLine(i, y1, i+1, y2, RED);
    
    // Y軸（青）
    y1 = GRAPH_Y_POS + GRAPH_HEIGHT/2 - (int)(accBuffer[idx1][1] * GRAPH_HEIGHT/4);
    y2 = GRAPH_Y_POS + GRAPH_HEIGHT/2 - (int)(accBuffer[idx2][1] * GRAPH_HEIGHT/4);
    
    if (y1 < GRAPH_Y_POS) y1 = GRAPH_Y_POS;
    if (y1 > GRAPH_Y_POS + GRAPH_HEIGHT) y1 = GRAPH_Y_POS + GRAPH_HEIGHT;
    if (y2 < GRAPH_Y_POS) y2 = GRAPH_Y_POS;
    if (y2 > GRAPH_Y_POS + GRAPH_HEIGHT) y2 = GRAPH_Y_POS + GRAPH_HEIGHT;
    
    StickCP2.Lcd.drawLine(i, y1, i+1, y2, BLUE);
  }
}

// データ送信
void sendData() {
  if (btConnected) {
    if (sendRawData) {
      // 生の加速度データを送信
      String dataString = String("RAW,") + String(millis()) + "," + 
                          String(accX, 3) + "," + String(accY, 3) + "," + String(accZ, 3);
      SerialBT.println(dataString);
      Serial.print("BT Send: ");
      Serial.println(dataString);
    } else {
      // 解析済みの歩行ピッチデータを送信
      String dataString = String("BPM,") + String(millis()) + "," + String(currentBPM, 2);
      SerialBT.println(dataString);
      Serial.print("BT Send: ");
      Serial.println(dataString);
    }
  }
}

// ボタン処理
void handleButton() {
  // StickCP2.update()は常にloopの最初で呼ばれているので、ここでは呼ばない
  
  // この関数ではM5.BtnA/BtnBの代わりにシリアル入力で動作を切り替える
  // (実機でのボタン操作はクラッシュの原因になる可能性があるため)
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == 'r' || cmd == 'R') {
      sendRawData = true;
      Serial.println("Mode changed to: RAW DATA");
    } else if (cmd == 'b' || cmd == 'B') {
      sendRawData = false;
      Serial.println("Mode changed to: BPM");
    }
  }
  
  // 注意：正常にボタン処理が確認できた場合のみコメントを外して使用すること
  /*
  // StickCP2.update() がloopで呼ばれていることを前提とする
  if (M5.BtnA.wasPressed()) {
    sendRawData = !sendRawData;
    Serial.print("Mode changed to: ");
    Serial.println(sendRawData ? "RAW DATA" : "BPM");
  }
  
  if (M5.BtnB.wasPressed()) {
    // B ボタンの機能（例：キャリブレーションモードなど）
    Serial.println("Button B pressed - additional function");
  }
  */
}

// Bluetooth コールバック
void btCallback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
  if (event == ESP_SPP_SRV_OPEN_EVT) {
    btConnected = true;
    Serial.println("Bluetooth connected!");
  } else if (event == ESP_SPP_CLOSE_EVT) {
    btConnected = false;
    Serial.println("Bluetooth disconnected!");
  }
}