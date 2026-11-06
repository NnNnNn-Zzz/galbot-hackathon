# 语音素材（必须赛前生成并提交）

GalbotSDK **没有 TTS**，只能播放 **16000 Hz / 16-bit little-endian / 单声道** 的裸 PCM。
所以文案要**赛前**合成成 `.pcm` 放进这个目录，运行时由 `robot_tools/audio.py` 分块播放。

需要的文件（名字必须一致，`robot_tools/audio.py::PROMPTS` 里是白名单）：

| 文件 | 文案 |
|---|---|
| `cup_unsafe.pcm` | 检测到有杯子靠近桌子边缘，请注意安全。 |
| `moving.pcm` | 正在把杯子移到安全位置。 |
| `safe.pcm` | 杯子已移到安全位置，请放心。 |
| `need_help.pcm` | 我无法安全移动这个杯子，请人工协助。 |

## Windows（SAPI，直接输出 16k/16bit/mono，不需要 ffmpeg）

```powershell
Add-Type -AssemblyName System.Speech
$fmt = New-Object System.Speech.AudioFormat.SpeechAudioFormatInfo(
    16000,
    [System.Speech.AudioFormat.AudioBitsPerSample]::Sixteen,
    [System.Speech.AudioFormat.AudioChannel]::Mono)

$texts = @{
  'cup_unsafe' = '检测到有杯子靠近桌子边缘，请注意安全。'
  'moving'     = '正在把杯子移到安全位置。'
  'safe'       = '杯子已移到安全位置，请放心。'
  'need_help'  = '我无法安全移动这个杯子，请人工协助。'
}

$synth = New-Object System.Speech.Synthesis.SpeechSynthesizer
$synth.Rate = 0
$synth.Volume = 100
foreach ($key in $texts.Keys) {
    $synth.SetOutputToWaveFile("$PWD\$key.wav", $fmt)
    $synth.Speak($texts[$key])
}
$synth.Dispose()

# wav -> 裸 pcm：去掉 44 字节 WAV 头
Get-ChildItem *.wav | ForEach-Object {
    $bytes = [System.IO.File]::ReadAllBytes($_.FullName)
    [System.IO.File]::WriteAllBytes(($_.BaseName + '.pcm'), $bytes[44..($bytes.Length - 1)])
    Remove-Item $_.FullName
}
```

## Linux（espeak-ng + sox）

```bash
for pair in \
  "cup_unsafe:检测到有杯子靠近桌子边缘，请注意安全。" \
  "moving:正在把杯子移到安全位置。" \
  "safe:杯子已移到安全位置，请放心。" \
  "need_help:我无法安全移动这个杯子，请人工协助。"
do
  key="${pair%%:*}"; text="${pair#*:}"
  espeak-ng -v cmn -s 150 -w "$key.wav" "$text"
  sox "$key.wav" -r 16000 -c 1 -s -2 -t raw "$key.pcm"
  rm -f "$key.wav"
done
```

## 校验

```bash
ls -l *.pcm                      # 每 1 秒 ≈ 32000 字节，太长说明采样率不对
python -c "import wave"          # 不需要，我们是裸 PCM
```

## 行为说明

- 素材**缺失**时不会报错崩掉：`speak` 会把文案打到终端并在 `data.mode` 标成
  `text_fallback`，同时 `played=false`。**不会假装播过。**
- 播放时按 2560 字节（= 80 ms）分块 + `sleep(0.05)`，播完再返回，
  避免"一边说话一边动"。
