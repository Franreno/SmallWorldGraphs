from small_world import SmallWorld


def main() -> None:
    sm = SmallWorld(100, 4, 0.1)
    print(sm.toString())


if __name__ == "__main__":
    main()
