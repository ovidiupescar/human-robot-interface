# Speech-to-text transcript with detected language.
# Published on /perception/transcript (final) and /perception/transcript_partial (streaming).

string text                       # the transcribed text
string language                   # BCP-47 code: "ro", "en", "" if unknown
float32 language_confidence       # 0.0-1.0; 0.0 if not estimated
bool is_final                     # true for /perception/transcript, false for partials
builtin_interfaces/Time stamp     # capture/finalization time
