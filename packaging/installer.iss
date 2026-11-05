; Inno Setup script for the Shock Absorber Monitor (built by build_windows.bat).
; Installs dist\ShockMonitor\ into Program Files with Start menu / desktop shortcuts.

#ifndef AppVersion
  #define AppVersion "1.1.0"
#endif

[Setup]
AppId={{6F1C2B7E-4E9A-4C55-9B8E-5A1D3C0F7A21}
AppName=Shock Absorber Monitor
AppVersion={#AppVersion}
AppPublisher=Shock Absorber Monitor
DefaultDirName={autopf}\Shock Absorber Monitor
DefaultGroupName=Shock Absorber Monitor
UninstallDisplayIcon={app}\ShockMonitor.exe
OutputDir=..\dist\installer
OutputBaseFilename=ShockMonitor-Setup-{#AppVersion}
SetupIconFile=icon.ico
Compression=lzma2
SolidCompression=yes
ArchitecturesAllowed=x64compatible
ArchitecturesInstallIn64BitMode=x64compatible
PrivilegesRequiredOverridesAllowed=dialog
WizardStyle=modern

[Tasks]
Name: "desktopicon"; Description: "{cm:CreateDesktopIcon}"; GroupDescription: "{cm:AdditionalIcons}"

[Files]
Source: "..\dist\ShockMonitor\*"; DestDir: "{app}"; Flags: ignoreversion recursesubdirs createallsubdirs
Source: "..\README.md"; DestDir: "{app}"; Flags: ignoreversion

[Icons]
Name: "{group}\Shock Absorber Monitor"; Filename: "{app}\ShockMonitor.exe"
Name: "{group}\Shock Absorber Monitor (simulator)"; Filename: "{app}\ShockMonitor.exe"; Parameters: "--simulate"
Name: "{group}\Recordings folder"; Filename: "{userdocs}\ShockMonitor\recordings"
Name: "{group}\Uninstall Shock Absorber Monitor"; Filename: "{uninstallexe}"
Name: "{autodesktop}\Shock Absorber Monitor"; Filename: "{app}\ShockMonitor.exe"; Tasks: desktopicon

[Dirs]
Name: "{userdocs}\ShockMonitor\recordings"

[Run]
Filename: "{app}\ShockMonitor.exe"; Description: "{cm:LaunchProgram,Shock Absorber Monitor}"; Flags: nowait postinstall skipifsilent
