from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
ACTIVE_DOCS = (
    "docs/OPERATIONS.md",
    "docs/SPEC.md",
    "docs/PLATFORM_DESIGN.md",
    "docs/IMPLEMENTATION_PLAN.md",
    "docs/ACCEPTANCE.md",
    ".env.example",
)


def test_active_docs_use_the_single_gateway_mapping() -> None:
    readme = (ROOT / "README.md").read_text(encoding="utf-8")
    operations = (ROOT / "docs/OPERATIONS.md").read_text(encoding="utf-8")

    assert "LocalForward 18090 127.0.0.1:18090" in operations
    assert "LocalForward 18089" not in operations
    assert "127.0.0.1:18089" not in operations
    assert "http://127.0.0.1:18090" in readme
    assert "docs/OPERATIONS.md" in readme


def test_active_runtime_docs_have_no_host_specific_workspace_path() -> None:
    for relative in ACTIVE_DOCS:
        text = (ROOT / relative).read_text(encoding="utf-8")
        assert "/home/zjc/workspace" not in text, relative


def test_duplicate_compose_documents_are_removed() -> None:
    obsolete = (
        "docs/superpowers/specs/2026-08-03-compose-managed-runtime-design.md",
        "docs/superpowers/plans/2026-08-03-compose-managed-runtime-implementation.md",
    )
    for relative in obsolete:
        assert not (ROOT / relative).exists(), relative


def test_process_documents_are_not_duplicated_at_repository_root() -> None:
    root_documents = (
        "OPERATIONS.md",
        "SPEC.md",
        "PLATFORM_DESIGN.md",
        "IMPLEMENTATION_PLAN.md",
        "ACCEPTANCE.md",
    )
    for relative in root_documents:
        assert not (ROOT / relative).exists(), relative
