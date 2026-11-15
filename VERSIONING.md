# IO Module sürüm yönetimi (makro / mikro)

myhexnet / Cuvals takip modeliyle aynı dil:

| Kavram | Örnek | Anlam |
|--------|--------|--------|
| **Makro** | `v0.60` | Ürün çizgisi / hat (`line` = `v0-60`) |
| **Mikro** | `v0.60-0001` | Anlamlı değişiklik / derleme takip numarası |
| **SemVer** | `0.60.0` | CMake / `esp_app_desc.version` |

## Dosyalar

- `VERSION.json` — tek kaynak (SoT)
- `changelog.json` — mikro geçmişi
- `VERSION` — makro (legacy)
- `main/hexnet_version.h` — `HEXNET_IO_RELEASE` / `HEXNET_IO_MICRO`
- `scripts/Sync-VersionFiles.ps1` — JSON → header/CMake
- `scripts/Bump-Version.ps1` — `-Bump micro|macro`

## Kullanım

```powershell
# Her derlemeden once (build.cmd otomatik cagirir)
.\scripts\Sync-VersionFiles.ps1

# Anlamli degisiklikten sonra
.\scripts\Bump-Version.ps1 -Bump micro -Notes "MQTT lifecycle fix"
.\scripts\Bump-Version.ps1 -Bump macro -Notes "Yeni hat v0.61"
```

`build.cmd` Sync’i otomatik çalıştırır.
