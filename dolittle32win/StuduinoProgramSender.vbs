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



'�B�擾�����R�}���h���C������2��葽���Ƃ����G���[
Dim INO_FILENAME 
INO_FILENAME = "\studuino\studuino.ino"
If oParam.Count > 0 Then

   INO_FILENAME = oParam(0)

End If



Dim ret

ret = HasStuduinoFoldor()
If ret = False Then
    'WScript.StdOut.WriteLine "Arduino�R���p�C�������w��̃t�H���_�ɑ��݂��܂���"
    WScript.Quit(1)
End If

'WScript.StdOut.WriteLine "Arduino�R���p�C�������w��̃t�H���_�ɑ��݂��邱�Ƃ��m�F"

dim com_port
com_port = CheckConnectStuduino()
If (com_port = "Err") Then
    'WScript.StdOut.WriteLine  "������Studuino�̐ڑ����m�F���܂���"
    'WScript.StdOut.WriteLine  "�s�v��USB�@������O���A�Ď��s���Ă�������"
    WScript.Quit(2)
End If

If (com_port = "Zero") Then
    'WScript.StdOut.WriteLine  "Studuino���ڑ�����Ă��܂���"
    'WScript.StdOut.WriteLine  "Studuino��ڑ����A�Ď��s���Ă�������"
    WScript.Quit(3)
End If

'WScript.StdOut.WriteLine "Studuino�Ɠ����VID�����PID������USB�@��̐ڑ����m�F"

'WScript.StdOut.WriteLine "�\�[�X�t�@�C���̃R�s�["
ret = CopySourceFile()
If (ret <> 0) Then
	'WScript.StdOut.WriteLine "�\�[�X�t�@�C���̃R�s�[�Ɏ��s�������ߏI�����܂�"
	'MsgBox (7)
    WScript.Quit(7)
End If

'WScript.StdOut.WriteLine "�R���p�C�����s"
ret = CompileSource(buildpath + "\studuino.cpp")
If (ret <> 0) Then
	'WScript.StdOut.WriteLine "�R���p�C���Ɏ��s�������ߏI�����܂�"
'    MsgBox (8)
    WScript.Quit(8)
End If


'WScript.StdOut.WriteLine "�����N���s"
ret = LinkObjects(buildpath + "\studuino.cpp")
If (ret <> 0) Then
	'WScript.StdOut.WriteLine "�����N�Ɏ��s�������ߏI�����܂�"
    'MsgBox (9)
    WScript.Quit(9)
End If

WScript.StdOut.WriteLine "objcopy���s"
ret = ObjCopy(buildpath + "\studuino.cpp")
If (ret <> 0) Then
	WScript.StdOut.WriteLine "objcopy�Ɏ��s�������ߏI�����܂�"
'    MsgBox (7)
    WScript.Quit(7)
End If

WScript.StdOut.WriteLine "hex�쐬"
ret = CreateHex(buildpath + "\studuino.cpp")
If (ret <> 0) Then
	'WScript.StdOut.WriteLine "hex�t�@�C���̍쐬�Ɏ��s�������ߏI�����܂�"
''    MsgBox (10)
    WScript.Quit(10)
End If

'WScript.StdOut.WriteLine "Studuino��ւ̃v���O�����]�����J�n"
ret = SendProgram(com_port ,buildpath + "\studuino.cpp")
If (ret <> 0) Then
	'WScript.StdOut.WriteLine "hex�t�@�C���̓]���Ɏ��s�������ߏI�����܂�"
    'MsgBox (5)
    WScript.Quit(5)
End If

'WScript.StdOut.WriteLine "�]���I��"

'WScript.StdOut.WriteLine "�r���h�t�H���_����ɂ���"
CleanBuildDir(buildpath)

WScript.Quit(0)

'------------------------------------------------------------
'------------------------------------------------------------
'------------------------------------------------------------




Function HasStuduinoFoldor()
	'�t�H���_�̊m�F
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
	'        MsgBox "����܂���"
	'     Else
	'        MsgBox "����܂���ł���"
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

	'��L��exe�t�@�C���ϊ���Ɏg���Ȃ����ߕύX
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