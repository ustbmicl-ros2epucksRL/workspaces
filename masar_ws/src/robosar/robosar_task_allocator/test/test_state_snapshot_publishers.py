from pathlib import Path
import sys
from unittest.mock import Mock, patch


ROBOSAR_SRC = Path(__file__).resolve().parents[2]
ALLOCATOR_SRC = ROBOSAR_SRC / "robosar_task_allocator" / "src"
FRONTIER_FILTER_SRC = (
    ROBOSAR_SRC
    / "robosar_task_generator"
    / "scripts"
    / "robosar_task_generator"
)
sys.path.insert(0, str(ALLOCATOR_SRC))
sys.path.insert(0, str(FRONTIER_FILTER_SRC))

import frontier_filter  # noqa: E402
import frontier_ta  # noqa: E402
import high_ta  # noqa: E402
import task_commander  # noqa: E402


def _publisher_kwargs(publisher, topic):
    for publisher_call in publisher.call_args_list:
        if publisher_call.args[0] == topic:
            return publisher_call.kwargs
    raise AssertionError("publisher for {!r} was not created".format(topic))


class _Commander(task_commander.TaskCommander):
    def execute(self):
        return None


def test_task_commander_exposes_current_assignments_and_completion_count_to_late_subscribers():
    rospack = Mock()
    rospack.get_path.return_value = "/tmp"
    publisher = Mock()

    with patch.object(task_commander.rospkg, "RosPack", return_value=rospack), patch.object(
        task_commander.rospy, "Publisher", publisher
    ):
        _Commander()

    assert _publisher_kwargs(publisher, "task_allocation")["latch"] is True
    assert _publisher_kwargs(publisher, "tasks_completed")["latch"] is True


def test_frontier_filter_exposes_latest_filtered_frontiers_to_late_subscribers():
    publisher = Mock()
    action_server = Mock()

    with patch.object(frontier_filter.rospy, "get_name", return_value="/frontier_filter"), patch.object(
        frontier_filter.rospy, "get_param", side_effect=lambda _name, default: default
    ), patch.object(frontier_filter.rospy, "Publisher", publisher), patch.object(
        frontier_filter.rospy, "spin"
    ), patch.object(
        frontier_filter.actionlib, "SimpleActionServer", return_value=action_server
    ):
        frontier_filter.FrontierFilter()

    assert _publisher_kwargs(
        publisher, "/frontier_filter/filtered_frontiers"
    )["latch"] is True


def test_frontier_allocator_exposes_latest_allocation_image_to_late_subscribers():
    publisher = Mock()

    with patch.object(frontier_ta.TaskCommander, "__init__", return_value=None), patch.object(
        frontier_ta.rospy, "get_param", side_effect=lambda _name, default: default
    ), patch.object(frontier_ta.rospy, "Timer"), patch.object(
        frontier_ta.rospy, "Rate"
    ), patch.object(frontier_ta.rospy, "Publisher", publisher), patch.object(
        frontier_ta.rospy, "Subscriber"
    ), patch.object(frontier_ta.tf, "TransformListener"), patch.object(
        frontier_ta, "CostCalculator"
    ):
        frontier_ta.FrontierAssignmentCommander()

    assert _publisher_kwargs(publisher, "task_allocation_image")["latch"] is True


def test_high_allocator_exposes_progress_and_information_map_to_late_subscribers():
    publisher = Mock()

    def initialize_frontier(instance):
        instance.utility_range = 1.5

    with patch.object(
        high_ta.FrontierAssignmentCommander,
        "__init__",
        initialize_frontier,
    ), patch.object(
        high_ta.rospy, "get_param", side_effect=lambda _name, default: default
    ), patch.object(high_ta, "get_map_tot_area_info", return_value=1.0), patch.object(
        high_ta.rospy, "Publisher", publisher
    ), patch.object(high_ta, "CostCalculator"):
        high_ta.HIGHAssignmentCommander()

    assert _publisher_kwargs(publisher, "/percent_completed")["latch"] is True
    assert _publisher_kwargs(publisher, "/information_map")["latch"] is True
