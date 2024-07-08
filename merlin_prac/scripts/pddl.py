#!/usr/bin/python3
from merlin2_basic_actions.merlin2_basic_types import (
    wp_type,
    object_type
)

from kant_dto import (
    PddlPredicateDto,
    PddlTypeDto
)


wp_checked = PddlPredicateDto("wp_checked", [wp_type])

sound_type = PddlTypeDto("sound")
sound_listened = PddlPredicateDto("sound_listened", [sound_type])
