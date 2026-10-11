"""Entry point for ``python -m payton``.

Prints Payton's AI-ready documentation so an assistant (or a curious human) can
learn the library without leaving the terminal::

    python -m payton                  # Markdown primer (payton.ai.context())
    python -m payton --catalog        # machine-readable API catalog as JSON
    python -m payton --examples       # list bundled, self-contained examples
    python -m payton --example NAME   # print one bundled example's source
    python -m payton --version        # installed version
"""

import sys

from payton.ai import catalog_json, context, example, examples, version

__all__ = ["main"]


def main(argv: list[str] | None = None) -> int:
    """Run the ``python -m payton`` command line interface."""
    args = sys.argv[1:] if argv is None else argv

    if "--version" in args or "-V" in args:
        print(version())
        return 0
    if "--catalog" in args:
        print(catalog_json())
        return 0
    if "--examples" in args:
        print("\n".join(examples()))
        return 0
    if "--example" in args:
        index = args.index("--example")
        if index + 1 >= len(args):
            print("usage: python -m payton --example NAME", file=sys.stderr)
            return 2
        try:
            print(example(args[index + 1]))
        except KeyError as exc:
            print(str(exc), file=sys.stderr)
            return 2
        return 0

    print(context())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
