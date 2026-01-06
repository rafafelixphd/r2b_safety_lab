import logging
import os
from typing import Optional, Dict

from rich.logging import RichHandler
# We import the custom components from handlers
from .handlers import EmojiRichHandler, get_console

# Global state to track initialization
_INITIALIZED = False
_DEFAULT_LEVEL = logging.INFO
_MODULE_LEVELS: Dict[str, int] = {}
_LOGGERS: Dict[str, logging.Logger] = {}

def _parse_env_config():
    """Parses environment variables to configure logging."""
    global _DEFAULT_LEVEL, _MODULE_LEVELS
    
    # Global Level
    env_level = os.getenv("R2B_LOG_LEVEL")
    if env_level:
        _DEFAULT_LEVEL = logging.getLevelName(env_level.upper())
        if not isinstance(_DEFAULT_LEVEL, int):
            _DEFAULT_LEVEL = logging.INFO
    
    # Flags/Module overrides
    flags_str = os.getenv("R2B_LOG_FLAGS", "")
    if flags_str:
        flags = [f.strip() for f in flags_str.split(",") if f.strip()]
        for flag in flags:
            parts = flag.split("=")
            if len(parts) == 2:
                mod_name, level_name = parts
                try:
                    level = logging.getLevelName(level_name.upper())
                    if isinstance(level, int):
                        _MODULE_LEVELS[mod_name] = level
                except (ValueError, TypeError):
                    pass
            elif len(parts) == 1:
                # Treat "module" as "module=DEBUG"
                mod_name = parts[0]
                _MODULE_LEVELS[mod_name] = logging.DEBUG


def setup_logger(level: Optional[int] = None):
    """Initializes the logging system with Custom RichHandler."""
    global _INITIALIZED
    if _INITIALIZED:
        return

    _parse_env_config()
    final_level = level if level is not None else _DEFAULT_LEVEL
    
    logging.basicConfig(
        level=final_level,
        format="%(message)s",
        datefmt="[%Y-%m-%d %H:%M:%S]", 
        handlers=[
            EmojiRichHandler(
                console=get_console(), 
                rich_tracebacks=True, 
                markup=True,
                show_path=True, 
            )
        ]
    )
    
    _INITIALIZED = True

def get_logger(name: str = "r2b", namespace: Optional[str] = None) -> logging.Logger:
    """
    Factory to get a logger with specific name and namespace.
    Format: [name] or [name/namespace]
    """
    # Auto-init if needed
    if not _INITIALIZED:
        setup_logger()

    full_name = name
    if namespace:
        full_name = f"{name}.{namespace}"
    
    if full_name in _LOGGERS:
        return _LOGGERS[full_name]
        
    logger = logging.getLogger(full_name)
    
    # Check for module-specific overrides
    if full_name in _MODULE_LEVELS:
        logger.setLevel(_MODULE_LEVELS[full_name])
    elif name in _MODULE_LEVELS:
        logger.setLevel(_MODULE_LEVELS[name])
    elif namespace and namespace in _MODULE_LEVELS:
        logger.setLevel(_MODULE_LEVELS[namespace])
        
    _LOGGERS[full_name] = logger
    return logger

def set_log_level(level: str, module: str = None):
    """
    Sets log level programmatically.
    """
    lvl = logging.getLevelName(level.upper())
    if not isinstance(lvl, int):
        return

    if module:
        logger = logging.getLogger(module)
        logger.setLevel(lvl)
        _MODULE_LEVELS[module] = lvl
    else:
        logging.getLogger().setLevel(lvl)
        global _DEFAULT_LEVEL
        _DEFAULT_LEVEL = lvl
