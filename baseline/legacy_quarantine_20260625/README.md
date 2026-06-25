# Legacy Quarantine 2026-06-25

This is a pointer for old experiment scaffolding that should not remain at repo
root once the latest baseline is verified.

The large local quarantine was moved outside this git repository:

```text
D:/vscode_dir/open_vins_legacy_quarantine_20260625
```

Policy:

- Keep: canonical source code, latest baseline config, canonical analysis tool.
- Quarantine: old one-off run scripts, obsolete build folders, ad-hoc result
  outputs, archived config packs, and exploratory packages.
- Delete later: only after the latest baseline runs and analyses pass on fly1,
  fly2, fly3, and fly4.

Nothing in that folder is treated as the current baseline, and it should not be
uploaded to GitHub.
