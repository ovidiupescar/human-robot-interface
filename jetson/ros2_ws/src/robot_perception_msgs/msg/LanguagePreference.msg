# Current resolved language preference for outgoing speech.
# Published latched on /language/current by LanguageResolver.

string language                   # "ro" | "en"
string source                     # "default" | "person" | "location" | "event" | "verbal_override"
                                  #   indicates which tier in the resolution stack chose this lang
string scope_id                   # optional: person_id, location_id, event_id for the binding source
builtin_interfaces/Time stamp
