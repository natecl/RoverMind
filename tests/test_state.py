from agent.state import extract_target


def test_extract_target_drive_to_the_x():
    assert extract_target("drive to the water bottle") == "water bottle"


def test_extract_target_go_to_the_x():
    assert extract_target("go to the red chair") == "red chair"


def test_extract_target_find_the_x():
    assert extract_target("find the laptop") == "laptop"


def test_extract_target_navigate_to_the_x():
    assert extract_target("navigate to the red chair") == "red chair"


def test_extract_target_approach_the_x():
    assert extract_target("approach the box") == "box"


def test_extract_target_move_to_the_x():
    assert extract_target("move to the water bottle") == "water bottle"


def test_extract_target_get_to_the_x():
    assert extract_target("get to the backpack") == "backpack"


def test_extract_target_trailing_punctuation_stripped():
    assert extract_target("drive to the water bottle.") == "water bottle"
    assert extract_target("go to the chair!") == "chair"


def test_extract_target_case_insensitive_match():
    assert extract_target("Drive To The Backpack") == "Backpack"


def test_extract_target_falls_back_to_full_task_if_no_pattern():
    # Unstructured task → return the trimmed task string itself so the
    # downstream LLM sees something usable rather than an empty target.
    assert extract_target("the green book please") == "the green book please"
