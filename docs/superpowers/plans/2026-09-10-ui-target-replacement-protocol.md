# UI Target Replacement Protocol Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Extend each UI `0x02` packet with a four-byte replaced target ID and reliably notify the UI when any RID moves between UI target identities.

**Architecture:** Keep RID/SORT association untouched. Add a small, independently tested sending-side state tracker in `core/main_tracking_v9.py`, then consult it only while building UI status packets. Update the local UI receiver and protocol documentation to the fixed 34-byte packet.

**Tech Stack:** Python 3, `struct`, UDP sockets, existing direct-call/pytest-style tests.

**Spec:** `docs/superpowers/specs/2026-09-10-ui-target-replacement-protocol-design.md`

## Global Constraints

- `0x02` format is exactly `!BB8sIffffI`, 34 bytes, with `replaced_target_id=0` meaning no deletion.
- All RID identities use replacement tracking; visual-only matches never create replacement events.
- Each queued old ID is carried by exactly three successful valid-distance UI sends.
- An ID currently bound to any RID cannot remain suppressed or queued for deletion.
- RID/SORT association, `ui_tracks`, control, strike, ranging, and `0x03` GPS logic remain unchanged.
- No backward compatibility with the old 30-byte UI status packet is required.

---

### Task 1: Encode and decode the 34-byte UI status packet

**Files:**
- Modify: `core/main_tracking_v9.py:1070-1120`
- Modify: `tools_py/udp_ui_receiver.py:7-22`
- Test: `tests/test_tracker_safety.py:345-405`

**Interfaces:**
- Consumes: existing `UISender.send_status(...)` arguments.
- Produces: `UISender.send_status(..., replaced_target_id: int = 0) -> bool` and receiver parsing of `replaced_target_id`.

- [ ] **Step 1: Write failing packet tests**

Update the valid-distance sender test to pass `replaced_target_id=23` and assert literal format `!BB8sIffffI`, length `34`, current ID `7`, and replaced ID `23`. Add a default-zero assertion for a call that omits the new argument. Add receiver assertions for the same literal packet.

- [ ] **Step 2: Run the focused tests and verify RED**

Run the focused tests directly with the existing lightweight SciPy import stub because the local Python lacks pytest/SciPy. Expected failure: `send_status()` rejects the new keyword or emits the old 30-byte packet.

- [ ] **Step 3: Implement minimal protocol changes**

Change packing to:

```python
struct.pack(
    "!BB8sIffffI",
    self.MSG_STATUS,
    int(camera_id),
    board_bytes,
    int(target_id),
    float(azimuth),
    float(elevation),
    float(distance),
    float(threat_score),
    int(replaced_target_id),
)
```

Validate `0 <= replaced_target_id <= 0xFFFFFFFF`; return `False` on invalid input. Update `tools_py/udp_ui_receiver.py` to require and unpack all nine values including threat score and replaced ID.

- [ ] **Step 4: Run focused tests and verify GREEN**

Expected: invalid distances remain blocked, valid packets are 34 bytes, replacement values round-trip, and default replacement is zero.

- [ ] **Step 5: Commit Task 1**

```text
feat: extend UI status packet with replaced target ID
```

### Task 2: Track RID-to-UI identity replacements independently

**Files:**
- Modify: `core/main_tracking_v9.py` near `UISender`
- Test: `tests/test_tracker_safety.py`

**Interfaces:**
- Produces: `RIDUIReplacementTracker.observe_bindings(bindings)`, `peek_replacement(rid_key)`, `mark_sent(rid_key, old_ui_id)`, `is_superseded(ui_id)`, and `forget(rid_key)`.

- [ ] **Step 1: Write failing state-transition tests**

Cover literal behaviors: first binding and same-ID recovery return zero; `7 -> 12` queues ID 7 with three sends; failed/no send leaves the count unchanged; two RID keys are isolated; `7 -> 12 -> 18` drains IDs 7 then 12 FIFO; and `forget()` removes state.

- [ ] **Step 2: Run focused tests and verify RED**

Expected failure: `RIDUIReplacementTracker` does not exist.

- [ ] **Step 3: Implement the minimal tracker**

Use ordinary dictionaries/lists only. `observe_bindings()` accepts a complete current-cycle mapping `{rid_key: ui_id}` so it can remove every active RID-bound UI ID from all pending queues and superseded sets after detecting transitions, making behavior independent of RID iteration order.

- [ ] **Step 4: Run state tests and verify GREEN**

Expected: every transition, queue, reactivation, and cleanup assertion passes.

- [ ] **Step 5: Commit Task 2**

```text
feat: track RID UI identity replacements
```

### Task 3: Integrate replacement state only into UI sending

**Files:**
- Modify: `core/main_tracking_v9.py:4070-4178, 6794-6895`
- Test: `tests/test_tracker_safety.py`
- Modify: `AGENTS.md`
- Modify: `DECISIONS.md`
- Modify: `PROJECT_CONTEXT.md`

**Interfaces:**
- Consumes: current `ui_fusion_pairs`, `final_distance_by_track`, `rid_item["key"]`, and `UISender.send_status()`.
- Produces: per-cycle current RID/UI binding registration, visual-only suppression, three-send acknowledgement, and permanent-RID cleanup.

- [ ] **Step 1: Write failing integration-helper tests**

Add a small pure helper around send eligibility so tests prove that a superseded visual-only ID is blocked, a RID-bound current ID is allowed, and a normal visual-only ID is allowed with `replaced_target_id=0`.

- [ ] **Step 2: Run focused tests and verify RED**

Expected failure: superseded UI IDs are not considered by current send eligibility.

- [ ] **Step 3: Wire state into the existing send loop**

Instantiate one tracker outside the main loop. Before status sends, batch-register every valid-distance RID pair and its assigned UI ID. For a RID pair, pass its queue head to `send_status()` and call `mark_sent()` only on success. For visual-only pairs, pass zero and skip any superseded UI ID. Call `forget()` only from the existing permanent RID deletion branch.

- [ ] **Step 4: Add diagnostic events**

Emit `RID_UI_ID_SWITCH`, `UI_REPLACEMENT_SEND`, `UI_REPLACEMENT_COMPLETE`, `UI_SUPERSEDED_TARGET_SKIP`, and `RID_UI_REPLACEMENT_STATE_DELETE` through the existing generic event logger without changing other CSV schemas.

- [ ] **Step 5: Update project documentation**

Record the 34-byte protocol, all-RID scope, visual exclusion, three-send retry, 3/12-second lifecycle interaction, and absence of hardware-control changes.

- [ ] **Step 6: Run focused and syntax verification**

Run direct focused tests, `python -m py_compile` for changed Python files, and `git diff --check`. If a full dependency environment is available, run the relevant pytest files; otherwise report the missing local pytest/SciPy limitation explicitly.

- [ ] **Step 7: Commit Task 3**

```text
feat: notify UI when RID target identity changes
```
