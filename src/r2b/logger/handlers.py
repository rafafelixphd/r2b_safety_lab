import logging
from rich.logging import RichHandler
from rich.console import Console
from rich.theme import Theme
from rich.text import Text

# Emoji mapping for log levels
_EMOJIS = {
    "DEBUG": "🔍",
    "INFO": "✨",
    "WARNING": "⚠️",
    "ERROR": "❌",
    "CRITICAL": "🔥"
}

# Custom theme (Rich style)
_THEME = Theme({
    "logging.level.debug": "cyan",
    "logging.level.info": "green",
    "logging.level.warning": "yellow",
    "logging.level.error": "red",
    "logging.level.critical": "red reverse",
    "logging.name": "blue",
    "logging.message": "white",
})

_CONSOLE = Console(theme=_THEME)

class EmojiRichHandler(RichHandler):
    """
    Custom RichHandler to inject emoji next to level name.
    """
    def get_level_text(self, record: logging.LogRecord) -> Text:
        """Get the level name text with unique styling and emoji."""
        level_name = record.levelname
        level_text = Text.styled(
            level_name.ljust(8), f"logging.level.{level_name.lower()}"
        )
        # Add emoji
        emoji = _EMOJIS.get(level_name, "")
        if emoji:
             # Prepend emoji
             level_text = Text(f"{emoji} ", style="default") + level_text
        return level_text

# Expose console for setup to use if needed, specifically for the handler
def get_console():
    return _CONSOLE
