# Raw numeric data. No physical units, axes or component semantics are implied.
uint8 MISSING=0
uint8 VALID=1
uint8 EMPTY=2
uint8 INVALID=3
uint8 status
# Validated logical shape, without squeezing singleton dimensions.
uint32[<=8] shape
bool shape_declared
# Original SDK shape declaration as JSON (also retained when validation fails).
string declared_shape_json
# NumPy dtype of the Python callback payload, NOT a claim about native SDK dtype.
string dtype
# C-order flattening, widened to float64 without physical conversion.
# INVALID and MISSING blocks carry no values; never interpret them as zero data.
float64[<=262144] raw_values
string error
