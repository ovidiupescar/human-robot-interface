"""KuzuDB schema for the robot knowledge graph.

Run init_schema() against a fresh Kuzu database. Idempotent — safe to call on
every node startup; existing tables aren't recreated.
"""

NODE_TYPES = [
    # --- the robot itself ---
    """
    CREATE NODE TABLE IF NOT EXISTS Self (
        id                STRING PRIMARY KEY,
        name              STRING,
        owner_id          STRING,
        preferences       STRING,
        born_at           TIMESTAMP,
        default_language  STRING,
        active_event_id   STRING
    )
    """,
    # --- people ---
    """
    CREATE NODE TABLE IF NOT EXISTS Person (
        id                  STRING PRIMARY KEY,
        primary_name        STRING,
        notes               STRING,
        created_at          TIMESTAMP,
        last_seen           TIMESTAMP,
        preferred_language  STRING
    )
    """,
    """
    CREATE NODE TABLE IF NOT EXISTS Alias (
        id            STRING PRIMARY KEY,
        person_id     STRING,
        alias         STRING,
        weight        DOUBLE
    )
    """,
    """
    CREATE NODE TABLE IF NOT EXISTS VoiceSample (
        id            STRING PRIMARY KEY,
        person_id     STRING,
        model         STRING,
        dim           INT32,
        embedding     FLOAT[],
        captured_at   TIMESTAMP
    )
    """,
    """
    CREATE NODE TABLE IF NOT EXISTS FaceSample (
        id            STRING PRIMARY KEY,
        person_id     STRING,
        model         STRING,
        dim           INT32,
        embedding     FLOAT[],
        captured_at   TIMESTAMP
    )
    """,
    # --- places ---
    """
    CREATE NODE TABLE IF NOT EXISTS Location (
        id                 STRING PRIMARY KEY,
        name               STRING,
        description        STRING,
        created_at         TIMESTAMP,
        language_override  STRING
    )
    """,
    """
    CREATE NODE TABLE IF NOT EXISTS LocationSample (
        id            STRING PRIMARY KEY,
        location_id   STRING,
        model         STRING,
        dim           INT32,
        embedding     FLOAT[],
        captured_at   TIMESTAMP,
        image_path    STRING
    )
    """,
    # --- memory ---
    """
    CREATE NODE TABLE IF NOT EXISTS Episode (
        id            STRING PRIMARY KEY,
        content       STRING,
        occurred_at   TIMESTAMP,
        duration_s    DOUBLE,
        embedding     FLOAT[],
        consolidated  BOOLEAN,
        created_at    TIMESTAMP
    )
    """,
    """
    CREATE NODE TABLE IF NOT EXISTS Fact (
        id            STRING PRIMARY KEY,
        content       STRING,
        confidence    DOUBLE,
        source        STRING,
        tags          STRING,
        embedding     FLOAT[],
        created_at    TIMESTAMP,
        last_referenced TIMESTAMP
    )
    """,
    """
    CREATE NODE TABLE IF NOT EXISTS Event (
        id                 STRING PRIMARY KEY,
        title              STRING,
        description        STRING,
        occurred_at        TIMESTAMP,
        duration_s         DOUBLE,
        created_at         TIMESTAMP,
        ends_at            TIMESTAMP,
        language_override  STRING
    )
    """,
    """
    CREATE NODE TABLE IF NOT EXISTS Topic (
        id            STRING PRIMARY KEY,
        label         STRING
    )
    """,
]

REL_TYPES = [
    # person <-> person
    "CREATE REL TABLE IF NOT EXISTS KNOWS (FROM Person TO Person, since TIMESTAMP, context STRING)",
    "CREATE REL TABLE IF NOT EXISTS PARENT_OF (FROM Person TO Person)",
    "CREATE REL TABLE IF NOT EXISTS SIBLING_OF (FROM Person TO Person)",
    "CREATE REL TABLE IF NOT EXISTS PARTNER_OF (FROM Person TO Person, since TIMESTAMP)",
    "CREATE REL TABLE IF NOT EXISTS COLLEAGUE_OF (FROM Person TO Person, org STRING)",
    "CREATE REL TABLE IF NOT EXISTS MENTOR_OF (FROM Person TO Person)",
    # person <-> samples
    "CREATE REL TABLE IF NOT EXISTS HAS_VOICE (FROM Person TO VoiceSample)",
    "CREATE REL TABLE IF NOT EXISTS HAS_FACE (FROM Person TO FaceSample)",
    "CREATE REL TABLE IF NOT EXISTS HAS_ALIAS (FROM Person TO Alias)",
    # locations
    "CREATE REL TABLE IF NOT EXISTS PART_OF (FROM Location TO Location)",
    "CREATE REL TABLE IF NOT EXISTS NEAR (FROM Location TO Location, distance_m DOUBLE)",
    "CREATE REL TABLE IF NOT EXISTS CONNECTS_TO (FROM Location TO Location)",
    "CREATE REL TABLE IF NOT EXISTS HAS_SAMPLE (FROM Location TO LocationSample)",
    # episodes / facts / events
    "CREATE REL TABLE IF NOT EXISTS OCCURRED_AT (FROM Episode TO Location)",
    "CREATE REL TABLE IF NOT EXISTS INVOLVES (FROM Episode TO Person)",
    "CREATE REL TABLE IF NOT EXISTS DURING (FROM Episode TO Event)",
    "CREATE REL TABLE IF NOT EXISTS TAGGED_WITH (FROM Episode TO Topic)",
    "CREATE REL TABLE IF NOT EXISTS DERIVED_FROM (FROM Fact TO Episode)",
    "CREATE REL TABLE IF NOT EXISTS ABOUT_PERSON (FROM Fact TO Person)",
    "CREATE REL TABLE IF NOT EXISTS ABOUT_LOCATION (FROM Fact TO Location)",
    "CREATE REL TABLE IF NOT EXISTS ABOUT_EVENT (FROM Fact TO Event)",
    "CREATE REL TABLE IF NOT EXISTS ABOUT_TOPIC (FROM Fact TO Topic)",
    "CREATE REL TABLE IF NOT EXISTS VISITED (FROM Person TO Location, when_ TIMESTAMP)",
    "CREATE REL TABLE IF NOT EXISTS ATTENDED (FROM Person TO Event)",
    "CREATE REL TABLE IF NOT EXISTS AT (FROM Event TO Location)",
    "CREATE REL TABLE IF NOT EXISTS ABOUT_SELF (FROM Fact TO Self)",
    "CREATE REL TABLE IF NOT EXISTS OWNED_BY (FROM Self TO Person)",
]


# Columns added after initial release. ALTER TABLE ADD will error if column
# already exists; we catch and ignore that error to keep init_schema idempotent
# across both fresh and existing databases.
COLUMN_ADDITIONS = [
    ("Self",     "default_language",   "STRING"),
    ("Self",     "active_event_id",    "STRING"),
    ("Person",   "preferred_language", "STRING"),
    ("Location", "language_override",  "STRING"),
    ("Event",    "ends_at",            "TIMESTAMP"),
    ("Event",    "language_override",  "STRING"),
]


def init_schema(conn):
    """Apply all CREATE statements. Idempotent."""
    for stmt in NODE_TYPES + REL_TYPES:
        conn.execute(stmt)
    # Best-effort column additions for pre-existing databases.
    for table, column, col_type in COLUMN_ADDITIONS:
        try:
            conn.execute(f"ALTER TABLE {table} ADD {column} {col_type}")
        except Exception:
            # Column already exists, or table not yet created. Either is fine.
            pass
