<#
.SYNOPSIS
    Git 一键提交并推送脚本（git add -A → git commit → 可选 git pull --rebase → git push -u origin HEAD）
.DESCRIPTION
    自动 add、commit 并 push 当前仓库的所有更改：
      git add -A → git commit -m <信息> →（-Pull 时）git pull --rebase → git push -u origin HEAD。
    若未提供提交信息，则使用当前时间戳；提交信息以 - 开头会被拒绝（怕被当成命令行参数）。
    脚本以"自身所在位置"定位目标仓库（git -C $PSScriptRoot rev-parse --show-toplevel）：
    脚本放在哪个仓库里，就更新哪个仓库；但若当前所在目录属于另一个 Git 仓库，
    脚本会拒绝执行（防误提交）。无 remote 时提交后直接报错退出，不会 push。
    注意：全程使用 git add -A，所有未被 .gitignore 忽略的改动（含未跟踪的新文件）都会入库。
.PARAMETER Message
    提交信息（可选，位置参数）
.PARAMETER Pull
    推送前先执行 git pull --rebase（开关）：在本地 commit 完成后、push 之前执行；
    变基失败会提示手动解决冲突并直接退出（不推送）。
.PARAMETER Force
    强制推送（开关，谨慎使用；实际命令为 git push -u origin HEAD --force-with-lease）
.PARAMETER Help
    显示帮助信息（可用 -h 简写）
.EXAMPLE
    .\一键更新.ps1 "修复登录 bug"
.EXAMPLE
    .\一键更新.ps1 -Pull
.EXAMPLE
    .\一键更新.ps1 -Pull -Force
.NOTES
    作者: 优化版 v2
    说明: 请以 UTF-8 with BOM 编码保存本文件，否则 Windows PowerShell 5.1
          下中文与 emoji 会显示为乱码甚至报语法错误。
#>

[CmdletBinding()]
param(
    [Parameter(Position = 0, HelpMessage = "提交信息（可选）")]
    [string]$Message = "",
    [switch]$Pull,
    [switch]$Force,
    [Alias('h')]
    [switch]$Help
)

# ---------- 帮助信息显示（放在最前面） ----------
if ($Help) {
    $helpText = @"

═══════════════════════════════════════════════════════════════
                    🚀 Git 一键更新助手 v2
═══════════════════════════════════════════════════════════════

【用 法】
  .\一键更新.ps1 [提交信息] [-Pull] [-Force] [-Help | -h]

【参数说明】
  [提交信息]   : 提交日志（可选），不填则自动生成时间戳
  -Pull        : 推送前先 git pull --rebase（拉取远程更新，避免冲突）
  -Force       : 强制推送（使用 --force-with-lease，远程有他人新提交时会拒绝）
  -Help / -h   : 显示本帮助信息

【使用示例】
  .\一键更新.ps1                               # 自动时间戳提交 + 推送
  .\一键更新.ps1 "修复登录模块的Bug"            # 带消息提交
  .\一键更新.ps1 -Pull                         # 先拉取再推送（推荐日常使用）
  .\一键更新.ps1 -Pull -Force                  # 拉取后强制推送（覆盖远程）

【安全设计】
  ✅ 以"脚本所在位置"定位仓库：脚本放进哪个仓库，就更新哪个仓库
  ✅ 若当前目录位于另一个 Git 仓库中，脚本会拒绝执行，防止误提交
  ✅ 无更改时智能退出；新分支首次推送自动设置上游（git push -u origin HEAD）
  ⚠️  强制推送会改写远程历史，请确保团队成员已知晓
  ⚠️  git add -A 会添加所有未忽略的文件，请先维护好 .gitignore

【首次使用提示】
  若提示"禁止运行脚本"，请执行：
    Set-ExecutionPolicy -Scope CurrentUser RemoteSigned

═══════════════════════════════════════════════════════════════
"@
    Write-Host $helpText -ForegroundColor Cyan
    exit 0
}

# ---------- 前置检查 ----------
if (-not (Get-Command git -ErrorAction SilentlyContinue)) {
    Write-Host "❌ 未检测到 git 命令，请先安装 Git：https://git-scm.com/downloads" -ForegroundColor Red
    exit 1
}

# ---------- 定位目标仓库（以脚本所在位置 $PSScriptRoot 为准，确定性行为；
#            脚本所在目录不在任何仓库内时报错退出） ----------
$repoRoot = git -C $PSScriptRoot rev-parse --show-toplevel 2>$null
if ($LASTEXITCODE -ne 0 -or -not $repoRoot) {
    Write-Host "❌ 脚本所在目录不在任何 Git 仓库中（未找到 .git）。" -ForegroundColor Red
    exit 1
}

# 安全检查：若当前目录位于另一个 Git 仓库中，拒绝执行，防止误提交
# （当前目录不在任何仓库时 git rev-parse 失败，这一项跳过、不阻拦）
$cwdRoot = git rev-parse --show-toplevel 2>$null
if ($LASTEXITCODE -eq 0 -and $cwdRoot) {
    if ((Resolve-Path $cwdRoot).Path -ine (Resolve-Path $repoRoot).Path) {
        Write-Host "❌ 当前目录位于另一个 Git 仓库中：$cwdRoot" -ForegroundColor Red
        Write-Host "   为避免误提交，请在目标仓库（脚本所在仓库）内运行本脚本。" -ForegroundColor Yellow
        exit 1
    }
}

# 切到目标仓库，之后所有 git 命令都作用于它；finally 里还原原目录
Push-Location $repoRoot
try {
    Write-Host "📍 目标仓库：$repoRoot" -ForegroundColor DarkGray
    $branch = git branch --show-current
    if ($branch) {
        Write-Host "🌿 当前分支：$branch" -ForegroundColor DarkGray
    }

    # ---------- 提交信息（缺省用时间戳；以 - 开头的消息会被拒绝） ----------
    if ([string]::IsNullOrWhiteSpace($Message)) {
        $Message = "自动更新: $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')"
    }
    elseif ($Message -match '^\s*-') {
        Write-Host "❌ 提交信息不能以 - 开头（会被误认为命令行参数）。" -ForegroundColor Red
        exit 1
    }

    # ---------- 显示当前变更 ----------
    Write-Host "`n📂 当前变更文件：" -ForegroundColor Cyan
    git status --short
    Write-Host ""

    # ---------- 无更改则提前退出 ----------
    if (-not (git status --porcelain)) {
        Write-Host "✅ 没有需要提交的更改，无需操作。" -ForegroundColor Green
        exit 0
    }

    # ---------- 添加所有更改（⚠ git add -A：未忽略的新增/修改/删除文件全部入库，先维护好 .gitignore） ----------
    Write-Host "➡️  添加所有更改（git add -A）..." -ForegroundColor Cyan
    git add -A
    if ($LASTEXITCODE -ne 0) {
        Write-Host "❌ git add 失败。" -ForegroundColor Red
        exit $LASTEXITCODE
    }

    # ---------- 提交 ----------
    Write-Host "📝 提交信息：$Message" -ForegroundColor Cyan
    git commit -m $Message
    if ($LASTEXITCODE -ne 0) {
        Write-Host "❌ 提交失败，请检查错误信息。" -ForegroundColor Red
        exit $LASTEXITCODE
    }
    $shortHash = git rev-parse --short HEAD
    Write-Host "✔ 已提交：$shortHash - $Message" -ForegroundColor DarkGray

    # ---------- 检查远程仓库（没配 remote 就直接退出，不做 push） ----------
    if (-not (git remote)) {
        Write-Host "❌ 未配置远程仓库。请先执行：git remote add origin <仓库地址>" -ForegroundColor Red
        exit 1
    }

    # ---------- 可选：拉取远程更新（rebase） ----------
    if ($Pull) {
        Write-Host "⬇️  正在拉取远程更新（git pull --rebase）..." -ForegroundColor Cyan
        git pull --rebase
        if ($LASTEXITCODE -ne 0) {
            Write-Host "⚠️  拉取/变基未完成（可能发生冲突）。" -ForegroundColor Yellow
            Write-Host "   请手动解决冲突后执行：" -ForegroundColor Yellow
            Write-Host "     git add <已解决的文件>" -ForegroundColor Yellow
            Write-Host "     git rebase --continue" -ForegroundColor Yellow
            Write-Host "   然后重新运行本脚本。" -ForegroundColor Yellow
            exit $LASTEXITCODE
        }
    }

    # ---------- 推送（git push -u origin HEAD：顺带把上游设为 origin/<当前分支>；
    #            -Force 时换成 --force-with-lease，远程有他人新提交会被拒绝） ----------
    if ($Force) {
        Write-Host "⚠️  强制推送已启用（--force-with-lease，远程若有他人新提交会被拒绝）！" -ForegroundColor Yellow
        git push -u origin HEAD --force-with-lease
    }
    else {
        git push -u origin HEAD
    }

    if ($LASTEXITCODE -eq 0) {
        Write-Host "`n✅ 一键更新完成！（$shortHash 已推送到 origin）" -ForegroundColor Green
    }
    else {
        Write-Host "`n❌ 推送失败，请检查网络、凭证或权限。" -ForegroundColor Red
        exit $LASTEXITCODE
    }
}
finally {
    Pop-Location
}
