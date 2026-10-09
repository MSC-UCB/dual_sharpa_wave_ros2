# Vendored Sharpa models

Source: https://github.com/sharpa-robotics/sharpa-urdf-usd-xml
Revision: `0d19cac602f46456b819e4b6a2c09a74982c9a3e`

Included: left/right `wave_01/{side}_sharpa_wave/{side}_sharpa_wave_with_flange.urdf`
and all 25 unique meshes referenced by each URDF (50 mesh files, approximately 14 MB).
URDFs are installed under `share/dual_sharpa_wave/urdf/`.

The only edits to the source URDFs are mesh URI replacements:
`package://{side}_sharpa_wave/meshes/` →
`package://dual_sharpa_wave/third_party/sharpa_models/{side}/meshes/`.
No joint names, axes, limits, inertias or geometry were altered.

The preview launch adds an isolated common root and fixed display offsets in memory,
then applies a TF prefix. These offsets are for display only, not CRX flange calibration.
Each flange URDF has 22 revolute joints, 35 links and no mimic joints.

Original LICENSE.txt and NOTICE.txt are included unchanged. Upstream specifies
Apache-2.0; the complete license text is also at the package root LICENSE.
SHA256SUMS.txt records the original URDF hashes (upstream: entries) and copied meshes.
