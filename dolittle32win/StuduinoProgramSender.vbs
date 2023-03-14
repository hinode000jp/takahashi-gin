'Option Explicit

Dim selftpath
selfpath = GetSelfDirectory()

Dim PROGRAM_PATH
PROGRAM_PATH = selfpath + "studuino\hardware\tools\avr\bin"

Dim libpath
libpath = "studuino\hardware\arduino\avr\build\studuino"

Dim buildpath
buildpath = "studuino\build"


Dim oParam

Set oParam = WScript.Arguments



'③取得したコマンドライン引数2つより多いときもエラー
Dim INO_FILENAME 
INO_FILENAME = "\studuino\studuino.ino"
If oParam.Count > 0 Then

   INO_FILENAME = oParam(0)

End If



Dim ret

ret = HasStuduinoFoldor()
If ret = False Then
    'WScript.StdOut.WriteLine "Arduinoコンパイル環境が指定のフォルダに存在しません"
    WScript.Quit(1)
End If

'WScript.StdOut.WriteLine "Arduinoコンパイル環境が指定のフォルダに存在することを確認"

dim com_port
com_port = CheckConnectStuduino()
If (com_port = "Err") Then
    'WScript.StdOut.WriteLine  "複数のStuduinoの接続を確認しました"
    'WScript.StdOut.WriteLine  "不要なUSB機器を取り外し、再実行してください"
    WScript.Quit(2)
End If

If (com_port = "Zero") Then
    'WScript.StdOut.WriteLine  "Studuinoが接続されていません"
    'WScript.StdOut.WriteLine  "Studuinoを接続し、再実行してください"
    WScript.Quit(3)
End If

'WScript.StdOut.WriteLine "Studuinoと同一のVIDおよびPIDをもつUSB機器の接続を確認"

'WScript.StdOut.WriteLine "ソースファイルのコピー"
ret = CopySourceFile()
If (ret <> 0) Then
	'WScript.StdOut.WriteLine "ソースファイルのコピーに失敗したため終了します"
	'MsgBox (7)
    WScript.Quit(7)
End If

'WScript.StdOut.WriteLine "コンパイル実行"
ret = CompileSource(buildpath + "\studuino.cpp")
If (ret <> 0) Then
	'WScript.StdOut.WriteLine "コンパイルに失敗したため終了します"
'    MsgBox (8)
    WScript.Quit(8)
End If


'WScript.StdOut.WriteLine "リンク実行"
ret = LinkObjects(buildpath + "\studuino.cpp")
If (ret <> 0) Then
	'WScript.StdOut.WriteLine "リンクに失敗したため終了します"
    'MsgBox (9)
    WScript.Quit(9)
End If

WScript.StdOut.WriteLine "objcopy実行"
ret = ObjCopy(buildpath + "\studuino.cpp")
If (ret <> 0) Then
	WScript.StdOut.WriteLine "objcopyに失敗したため終了します"
'    MsgBox (7)
    WScript.Quit(7)
End If

WScript.StdOut.WriteLine "hex作成"
ret = CreateHex(buildpath + "\studuino.cpp")
If (ret <> 0) Then
	'WScript.StdOut.WriteLine "hexファイルの作成に失敗したため終了します"
''    MsgBox (10)
    WScript.Quit(10)
End If

'WScript.StdOut.WriteLine "Studuino基板へのプログラム転送を開始"
ret = SendProgram(com_port ,buildpath + "\studuino.cpp")
If (ret <> 0) Then
	'WScript.StdOut.WriteLine "hexファイルの転送に失敗したため終了します"
    'MsgBox (5)
    WScript.Quit(5)
End If

'WScript.StdOut.WriteLine "転送終了"

'WScript.StdOut.WriteLine "ビルドフォルダを空にする"
CleanBuildDir(buildpath)

WScript.Quit(0)

'------------------------------------------------------------
'------------------------------------------------------------
'------------------------------------------------------------




Function HasStuduinoFoldor()
	'フォルダの確認
	Dim FS
	Set FS = CreateObject("Scripting.FileSystemObject")
	HasStuduinoFoldor = FS.FolderExists(PROGRAM_PATH)
	Set FS = Nothing
End Function


Function CompStuduino(strText)
	Dim objRE
	Set objRE = CreateObject("VBScript.RegExp")
	With objRE

	    .Pattern = "(COM\d*\s)- Prolific -\sUSB\\VID_067B&PID_2303.+"
	    .IgnoreCase = True
	    .Global = False

	    CompStuduino = .Test(strText)
	'     If CompStuduino = True Then
	'        MsgBox "ありました"
	'     Else
	'        MsgBox "ありませんでした"
	'     End If
	End With
	Set objRE = Nothing
End Function

Function CheckConnectStuduino()
    Dim WshShell, outExec, StdOut
    Dim strCmd
    Dim retStr
    retStr = Null
    
    strCmd = selfpath+"\studuino\hardware\tools\listComPorts.exe"

    Set WshShell = CreateObject("WScript.Shell")
    Set outExec = WshShell.Exec(strCmd)
    Do While outExec.Status = 0
         WScript.Sleep 100
    Loop
    Set StdOut = outExec.StdOut
    
    
    Dim count
    count = 0
    
    Do While Not StdOut.AtEndOfStream
        Dim outStr
        outStr = StdOut.ReadLine()
        If CompStuduino(outStr) Then
            Dim aryStr
            aryStr = Split(outStr)
            retStr = "" + aryStr(0)
            count = count + 1
        End If
    Loop
    
    If count = 1 Then
       CheckConnectStuduino = retStr
    ElseIf count > 1 Then
        CheckConnectStuduino = "Err"
    Else
        CheckConnectStuduino = "Zero"
    End If
     
    'Dim  StdErr
    'Dim  szStr
    'Set StdErr = outExec.StdErr
    'szStr = "STDERR" & vbCrLf
    'Do While Not StdErr.AtEndOfStream
    '   szStr = szStr & StdErr.ReadLine() & vbCrLf
    'Loop
    Set WshShell = Nothing
End Function

Function CopySourceFile()
	Dim FS
	Set FS = CreateObject("Scripting.FileSystemObject")

	CopySourceFile = FS.CopyFile(selfpath + INO_FILENAME , buildpath + "\studuino.cpp")
	Set FS = Nothing
End Function

Function CompileSource(filepath)
    Dim WshShell, outExec

    Dim strCmd
    strCmd = PROGRAM_PATH + "\avr-g++.exe"
    
    Dim strPrm
    strPrm = " -c -g -Os -w -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -MMD -mmcu=atmega168 -DF_CPU=8000000L -DARDUINO=10603 -DARDUINO_AVR_PRO -DARDUINO_ARCH_AVR" _
             + " -Istuduino\hardware\arduino\avr\variants\standard" _
             + " -Istuduino\hardware\arduino\avr\cores\arduino" _
             + " -Istuduino\hardware\arduino\avr\libraries\Wire\src" _
             + " -Istuduino\hardware\arduino\avr\libraries\Wire\src\utility" _
             + " -Istuduino\libraries\Servo\src" _
             + " -Istuduino\libraries\MMA8653" _
             + " -Istuduino\libraries\MPU6050" _
             + " -Istuduino\libraries\IRremoteForStuduino" _
             + " -Istuduino\libraries\ColorSensor" _
             + " -Istuduino\libraries\Studuino " _
             + filepath + " -o " + filepath + ".o"

    'WScript.StdOut.WriteLine "          "
    'WScript.StdOut.WriteLine strCmd + strPrm
    Set WshShell = CreateObject("WScript.Shell")
    Set outExec = WshShell.Exec(strCmd + strPrm)
    Do While outExec.Status = 0
         WScript.Sleep 100
    Loop
    VelifyCompile = outExec.ExitCode
    Set WshShell = Nothing

End Function

Function LinkObjects(filepath)
    Dim WshShell, outExec
    
    Dim strCmd
    strCmd = PROGRAM_PATH + "\avr-gcc.exe"
    
    Dim strPrm
    strPrm = " -w -Os -Wl,--gc-sections -mmcu=""atmega168"" " _
             + "-o " + filepath + ".elf " + filepath + ".o " _
             + libpath + "\studuino.a " + libpath + "\core.a " + "-lm"
    'WScript.StdOut.WriteLine strCmd + strPrm
    Set WshShell = CreateObject("WScript.Shell")
    Set outExec = WshShell.Exec(strCmd + strPrm)
    Do While outExec.Status = 0
         WScript.Sleep 100
    Loop
    VelifyCompile = outExec.ExitCode
    Set WshShell = Nothing

End Function

Function ObjCopy(filepath)
    Dim WshShell, outExec
    
    Dim strCmd
    strCmd = PROGRAM_PATH + "\avr-objcopy.exe"
    
    Dim strPrm
    strPrm = " -O ihex -j .eeprom --set-section-flags=.eeprom=alloc,load --no-change-warnings --change-section-lma .eeprom=0 " _
             + filepath + ".elf " + filepath + ".eep"
    'WScript.StdOut.WriteLine strCmd + strPrm
    Set WshShell = CreateObject("WScript.Shell")
    Set outExec = WshShell.Exec(strCmd + strPrm)
    Do While outExec.Status = 0
         WScript.Sleep 100
    Loop
    VelifyCompile = outExec.ExitCode
    Set WshShell = Nothing

End Function

Function CreateHex(filepath)
    Dim WshShell, outExec
    
    Dim strCmd
    strCmd = PROGRAM_PATH + "\avr-objcopy.exe"
    
    Dim strPrm
    strPrm = " -O ihex -R .eeprom " _
             + filepath + ".elf " + filepath + ".hex"
    'WScript.StdOut.WriteLine strCmd + strPrm
    Set WshShell = CreateObject("WScript.Shell")
    Set outExec = WshShell.Exec(strCmd + strPrm)
    Do While outExec.Status = 0
         WScript.Sleep 100
    Loop
    VelifyCompile = outExec.ExitCode
    Set WshShell = Nothing

End Function

Function SendProgram(com_port,filepath)
    Dim WshShell, outExec
    
    Dim confpath
    confFile = "studuino\hardware\tools\avr\etc\avrdude.conf"

    Dim strCmd
    strCmd = PROGRAM_PATH + "\avrdude.exe"
    
    Dim strPrm
    strPrm = " -q -q -C" + confFile + " -patmega168 -carduino -b115200 -D " _
             + "-P\\.\" + com_port + " -Uflash:w:" + filepath + ".hex:i"
    'WScript.StdOut.WriteLine strCmd + strPrm
    Set WshShell = CreateObject("WScript.Shell")
    Set outExec = WshShell.Exec(strCmd + strPrm)
    Do While outExec.Status = 0
         WScript.Sleep 100
    Loop
    SendProgram = outExec.ExitCode
    Set WshShell = Nothing

End Function

Function CleanBuildDir(filepath)
	Dim FS
	Set FS = CreateObject("Scripting.FileSystemObject")
	CleanBuildDir = FS.DeleteFile(filepath + "\*.*")
	Set FS = Nothing
End Function

Function GetSelfDirectory()

	'pathLen = Len(wscript.scriptfullname) - Len(wscript.scriptname)
	'GetSelfDirectory = Left(wscript.scriptfullname, pathLen)
         Dim aryStr
         aryStr = Split(ModulePath,"\")

	'上記はexeファイル変換後に使えないため変更
	dim name
	name = ModulePath
	pathLen = Len(ModulePath) - Len(aryStr(Ubound(aryStr)))
	GetSelfDirectory = Left(ModulePath, pathLen)
End Function


Function ModulePath()
  dim T,fHandle
  ModulePath=WScript.ScriptFullName
  T=lcase(ModulePath)
  T=left(T,len(T)-4)
  if right(T,4)<>".tmp" then exit function
  on error resume next
  set fHandle=CreateObject("Scripting.FileSystemObject").OpenTextFile(T)
  T=fHandle.ReadLine
  fHandle.close
  if err.number=0 then ModulePath=T
End Function