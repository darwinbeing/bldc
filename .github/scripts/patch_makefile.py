#!/usr/bin/env python3
"""
Version-independent Makefile patcher for BLDC firmware.

Change 1: Insert per-build artifact cleanup after each fw_$(1)_vescfw build rule.
Change 2: Remove the BUILD_CRUFT block from all_fw_package (if present).

Both changes are idempotent and match by content, not line number.
"""
import re
import sys

MAKEFILE = "Makefile"

CLEANUP_BLOCK = (
    "ifneq ($(OSFAMILY), windows)\n"
    "\t$(V1) [ ! -d \"$(BUILD_DIR)/$(1)/lst\" ] || $(RM) -r \"$(BUILD_DIR)/$(1)/lst\"\n"
    "\t$(V1) [ ! -d \"$(BUILD_DIR)/$(1)/obj\" ] || $(RM) -r \"$(BUILD_DIR)/$(1)/obj\"\n"
    "\t$(V1) [ ! -d \"$(BUILD_DIR)/$(1)/.dep\" ] || $(RM) -r \"$(BUILD_DIR)/$(1)/.dep\"\n"
    "else\n"
    "\t$(V1) powershell -noprofile -command \"& {if (Test-Path $(BUILD_DIR)/$(1)/lst)"
    " {Remove-Item -Recurse $(BUILD_DIR)/$(1)/lst}}\"\n"
    "\t$(V1) powershell -noprofile -command \"& {if (Test-Path $(BUILD_DIR)/$(1)/obj)"
    " {Remove-Item -Recurse $(BUILD_DIR)/$(1)/obj}}\"\n"
    "\t$(V1) powershell -noprofile -command \"& {if (Test-Path $(BUILD_DIR)/$(1)/.dep)"
    " {Remove-Item -Recurse $(BUILD_DIR)/$(1)/.dep}}\"\n"
    "endif\n"
)

with open(MAKEFILE, "r") as f:
    content = f.read()

changed = False

# Change 1: insert cleanup block after USE_VERBOSE_COMPILE=no
if "$(BUILD_DIR)/$(1)/lst" not in content:
    new_content = re.sub(
        r"([ \t]+build_args=.*USE_VERBOSE_COMPILE=no\n)",
        r"\g<1>" + CLEANUP_BLOCK,
        content,
    )
    if new_content != content:
        content = new_content
        changed = True
        print("[OK] Added per-build cleanup block")
    else:
        print("[WARN] USE_VERBOSE_COMPILE=no not found — skipped change 1", file=sys.stderr)
        sys.exit(1)
else:
    print("[SKIP] Per-build cleanup already present")

# Change 2: remove BUILD_CRUFT block (pattern matches both \t and space-indented forms)
if "BUILD_CRUFT" in content:
    new_content = re.sub(
        r"# Find all the leftover object and lst files\n"
        r"[ \t]*\$\(eval BUILD_CRUFT[^\n]*\)\n"
        r"\n"
        r"# Delete the cruft[^\n]*\n"
        r"ifneq[^\n]*\n"
        r"[ \t]*[^\n]*\n"
        r"else\n"
        r"[ \t]*[^\n]*\n"
        r"endif\n",
        "",
        content,
    )
    if new_content != content:
        content = new_content
        changed = True
        print("[OK] Removed BUILD_CRUFT block")
    else:
        print("[WARN] BUILD_CRUFT pattern did not match — skipped change 2", file=sys.stderr)
else:
    print("[SKIP] BUILD_CRUFT not present")

if changed:
    with open(MAKEFILE, "w") as f:
        f.write(content)
    print("Makefile updated successfully.")
else:
    print("No changes needed.")
