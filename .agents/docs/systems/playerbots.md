# Playerbots integration

Mandatory when touching a core file that carries `#ifdef MOD_PLAYERBOTS`, the `PlayerbotScript` hook
type, or `PlayerbotsDatabase`. Covers **this repository's** surface only; module internals live in
`modules/mod-playerbots/docs/` (gitignored here — `/modules/*`).

mod-playerbots is not an ordinary module. `modules/CMakeLists.txt` special-cases it by name, and the
core carries ~30 `#ifdef MOD_PLAYERBOTS` sites on its behalf.

## Build

`modules/CMakeLists.txt` sets `-DMOD_PLAYERBOTS` on `database` (PRIVATE), `game-interface`
(**INTERFACE**, so it reaches all core) and `modules` (PRIVATE).

Consequences: core constants and vtables (`IWorld`, `PathGenerator`) change with the module present;
a worldserver binary is not module-agnostic; `DISABLED_AC_MODULES` does not just drop a module, it
changes core behaviour. Adding or removing it needs a full core rebuild.

## Hook type

| Piece | Location |
|---|---|
| `PlayerbotScript` declaration | `src/server/game/Scripting/ScriptMgr.h:109` |
| Registration | `src/server/game/Scripting/ScriptMgr.cpp:82` |
| Dispatch | `src/server/game/Scripting/ScriptDefines/PlayerbotsScript.cpp` |

| Hook | Call site |
|---|---|
| `OnPlayerbotUpdate` | `World.cpp:1202` |
| `OnPlayerbotPacketSent` | `WorldSession.cpp:316` |
| `OnPlayerbotUpdateSessions` | `WorldSession.cpp:616` |
| `OnPlayerbotLogout` | `WorldSession.cpp:721` |
| `OnPlayerbotLogoutBots` | `WorldSessionMgr.cpp:234` |
| `OnPlayerbotCheckUpdatesToSend` | `Map.cpp:1724` |
| `OnPlayerbotCheckLFGQueue` | `LFGQueue.cpp:415` |
| `OnPlayerbotCheckKillTask` | `Unit.cpp:14251` |
| `OnPlayerbotCheckPetitionAccount` | `PetitionsHandler.cpp:497` |

## Core patches

| Location | Change | Do not |
|---|---|---|
| `Map.cpp:1724` | `SendObjectUpdates` skips + `Clear()`s update data for bots | Remove. It is why 500 bots are affordable — bots pay zero packet build/encrypt/socket cost. |
| `PathGenerator.h:35` | `MAX_PATH_LENGTH` / `MAX_POINT_PATH_LENGTH` 74 → 148 | Assume 74 in path-length maths. |
| `PathGenerator.cpp:773` | Bot-only `dtQueryFilter` — excludes `NAV_MAGMA`/`NAV_SLIME`/`NAV_GROUND_STEEP`, water area cost 20 | Assume bots and players path identically. |
| `MotionMaster.h:268` | `MoveKnockbackFromForPlayer`, `MovePointBackwards` | — |
| `WorldSession.cpp:472-525` | 4× `sScriptMgr->OnPacketReceived` under `#ifdef` | Drop when refactoring the opcode switch. |
| `Group.h:274` | `GetTargetIcon` accessor | — |
| `ChatHandler.cpp:352` | Hyperlink validation bypass for `Hfound:` | — |
| `IWorld.h:108`, `World.h:232`/`:311` | `GetPlayerbotsDBRevision` | — |

## Bots are Players on socket-less sessions

`modules/mod-playerbots/src/Bot/PlayerbotMgr.cpp:203` constructs
`new WorldSession(accountId, "", 0x0, nullptr, SEC_PLAYER, ...)` — null socket. Bots use the real
login holder, `Player`, movement, spell and loot paths. Discriminator: `WorldSession::IsBot()`
(`WorldSession.h:1230`).

Implication for core work: a `Player*` may have no socket. Code reached from a `Player` must not
assume `GetSession()->GetRemoteAddress()`, a live `WorldSocket`, or that sending a packet reaches
anything.

## Fourth database

`PlayerbotsDatabase` is a full `DatabaseWorkerPool` peer of World/Character/Login.

`DatabaseEnv.h:43` (accessor) · `DatabaseEnvFwd.h:35,48,81,96` · `DatabaseLoader.h:51`
(`DATABASE_PLAYERBOTS = 8`, folded into `DATABASE_MASK_ALL`) · `DatabaseLoader.cpp:242` ·
`DatabaseWorkerPool.cpp:44,579` · `DBUpdater.cpp:200,619` · `cs_server.cpp:218` (`.server info`).

Adding a database enum or worker-pool template needs the `#ifdef` branch updated too.

## Threading

Bot AI runs on **map threads**. Group/LFG/guild/BG mutation is queued as `PlayerbotOperation` and
drained on the **world thread** from `WorldScript::OnUpdate`. If you add a core API that bots call
and that is not map-thread-safe, say so — the module cannot infer it.

## Related

- `src/server/game/TC9Sidecar/` — the other external-process integration. Its pump is gated on
  `Cluster.Enabled` (`World.cpp:1349`) and its stub `exit()`s on call; do not assume it is available.
- `modules/mod-playerbots/docs/` — module internals, coupling audit, sidecar design.
