# Request to synthesize speech.
# Published on /tts/say (full sentence) and /tts/stream_in (text chunks).

string text                       # text to speak (RO or EN)
string language                   # "ro" | "en" | "" (auto-resolve via LanguageResolver)
string voice                      # optional voice id override; "" = use default for language
                                  #   examples: "af_bella" (Kokoro EN), "ro_RO-mihai-medium" (Piper RO)
