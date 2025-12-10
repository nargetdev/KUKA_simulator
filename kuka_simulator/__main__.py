from __future__ import annotations

import asyncio

from .cli import run_from_cli


def main() -> None:
    asyncio.run(run_from_cli())


if __name__ == "__main__":
    main()

