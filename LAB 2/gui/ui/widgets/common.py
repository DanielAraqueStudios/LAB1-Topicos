"""Small shared UI helpers."""
from __future__ import annotations

from PyQt6.QtWidgets import QFrame, QLabel, QPushButton, QVBoxLayout, QWidget


def set_prop(w: QWidget, name: str, value) -> None:
    """Change a dynamic property and re-apply the stylesheet."""
    w.setProperty(name, value)
    w.style().unpolish(w)
    w.style().polish(w)


def make_card(title: str) -> tuple[QFrame, QVBoxLayout]:
    card = QFrame()
    card.setObjectName("Card")
    lay = QVBoxLayout(card)
    lay.setContentsMargins(14, 12, 14, 14)
    lay.setSpacing(10)
    if title:
        t = QLabel(title.upper())
        t.setObjectName("CardTitle")
        lay.addWidget(t)
    return card, lay


def make_button(text: str, variant: str = "", tip: str = "") -> QPushButton:
    b = QPushButton(text)
    if variant:
        b.setProperty("variant", variant)
    if tip:
        b.setToolTip(tip)
    return b


def make_badge(text: str = "", status: str = "none") -> QLabel:
    b = QLabel(text)
    b.setProperty("badge", status)
    return b
