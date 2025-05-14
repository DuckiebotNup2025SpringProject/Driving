#!/bin/python3
import os

import networkx as nx
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MapNode(Node):
    def __init__(self):
        try:
            bot_name = os.getenv('VEHICLE_NAME')
        except Exception:
            self.get_logger().error('Bot name not specified')
            bot_name = "example_robot"
        super().__init__('map_node')
        self.bot_name = bot_name
        self.apriltag_topic = f'/{bot_name}/apriltag'
        self.route_topic = f'/{bot_name}/route'
        self.actual_tag = 243

        self.apriltag_subscription = self.create_subscription(
            String,
            self.apriltag_topic,
            self.parse_tag,
            10)

        self.route_publisher = self.create_publisher(
            String,
            self.route_topic,
            10)

        self.get_logger().info('Bot name: ' + bot_name)
        self.timer = self.create_timer(5, lambda: self.find_intersection_by_tag(self.actual_tag))

        self.route = "AF"

        self.town_map = nx.Graph()

        self.town_map.add_node('A', tags=[148, 145, 146, 151])
        self.town_map.add_node('B', tags=[147, 137, 0, 160])
        self.town_map.add_node('J', tags=[134, 0, 159, 243])
        self.town_map.add_node('D', tags=[0, 156, 149, 238])
        self.town_map.add_node('E', tags=[245, 150, 244, 0])

        self.town_map.add_edge('A', 'E', first_tag='151', second_tag='150')
        self.town_map.add_edge('A', 'D', first_tag='148', second_tag='149')
        self.town_map.add_edge('A', 'J', first_tag='145', second_tag='243')
        self.town_map.add_edge('A', 'B', first_tag='146', second_tag='147')
        self.town_map.add_edge('B', 'E', first_tag='160', second_tag='244')
        self.town_map.add_edge('E', 'D', first_tag='245', second_tag='238')
        self.town_map.add_edge('J', 'D', first_tag='134', second_tag='156')
        self.town_map.add_edge('J', 'B', first_tag='159', second_tag='137')

        self.send_route()

    def send_route(self):
        msg = String()
        msg.data = self.run_navigation()
        self.route_publisher.publish(msg)

    def parse_tag(self, msg):
        try:
            tag = msg.data
            if msg is None:
                return
            self.actual_tag = int(tag)
        except Exception:
            self.get_logger().error("Error parsing tag")

    def find_edge_by_tag(self, graph, tag_value):
        tag_str = str(tag_value)
        for u, v, attrs in graph.edges(data=True):
            found_vertices = []
            if attrs.get('first_tag') == tag_str:
                found_vertices.append(u)
            if attrs.get('second_tag') == tag_str:
                found_vertices.append(v)
            if found_vertices:
                return (u, v), found_vertices

        return None, None

    def find_edges_from_vertex(self, graph, vertex):
        vertex_tags = graph.nodes[vertex].get('tags', [])
        result = {}

        for tag in vertex_tags:
            tag_str = str(tag)
            matches = []
            for u, v, attrs in graph.edges(vertex, data=True):
                if attrs.get('first_tag') == tag_str or attrs.get('second_tag') == tag_str:
                    adjacent = v if u == vertex else u
                    matches.append(((u, v), adjacent))
            result[tag] = matches if matches else None

        return result

    def compute_turns(self, vertex, incoming_tag):
        tags = self.town_map.nodes[vertex].get('tags', [])
        try:
            i = tags.index(incoming_tag)
        except ValueError:
            self.get_logger().info(f"Error: the incoming tag {incoming_tag} was not found in vertex {vertex}!")
            return None

        turns = {
            0: tags[(i + 2) % 4],
            90: tags[(i + 3) % 4],
            180: tags[i],
            270: tags[(i + 1) % 4]
        }
        return turns

    def find_intersection_by_tag(self, tag_to_find):
        edge, vertices_with_tag = self.find_edge_by_tag(self.town_map, tag_to_find)
        self.get_logger().info(
            "INFO FOR TAG " + str(tag_to_find) + "___________________________________________________")
        if edge:
            selected_vertex = vertices_with_tag[0]
            incoming_tag = int(tag_to_find)
            turns = self.compute_turns(selected_vertex, incoming_tag)
            edges_mapping = self.find_edges_from_vertex(self.town_map, selected_vertex)
            self.get_logger().info(
                f"\nYou have arrived at intersection {selected_vertex} via road {edge}. From this intersection, you can proceed to the following:")

            for tag, infos in edges_mapping.items():
                if infos is not None:
                    turn_angle = None
                    for angle, t in turns.items():
                        if t == tag:
                            turn_angle = angle
                            break
                    for edge_info in infos:
                        edge_data, adjacent = edge_info
                        if turn_angle is not None:
                            self.get_logger().info(
                                f"Tag {tag} leads to intersection {adjacent} - turn {turn_angle} degrees clockwise.")
        else:
            self.get_logger().error(f"No edge with tag {tag_to_find} was found.")

    def find_shortest_path_instructions(self, graph, start, end):
        """
        Returns a list of commands for the robot:
          "T<angle>" — turn by <angle> degrees,
          "S<tag>"   — drive until encountering tag <tag> at the next intersection.
        """
        if start not in graph.nodes:
            raise ValueError(f"Node '{start}' not found")
        if end not in graph.nodes:
            raise ValueError(f"Node '{end}' not found")

        path = nx.shortest_path(graph, source=start, target=end, weight='weight')
        actions = []
        incoming_tag = None

        for u, v in zip(path, path[1:]):
            attrs = graph[u][v]
            # Determine which tag is on the u side and which on the v side
            if str(attrs['first_tag']) in map(str, graph.nodes[u]['tags']):
                depart_tag = int(attrs['first_tag'])
                arrive_tag = int(attrs['second_tag'])
            else:
                depart_tag = int(attrs['second_tag'])
                arrive_tag = int(attrs['first_tag'])

            # 1) Turn
            if incoming_tag is None:
                angle = 0
            else:
                turns = self.compute_turns(u, incoming_tag)
                angle = next((ang for ang, t in turns.items() if t == depart_tag), 0)
            actions.append(f"T{angle}")

            # 2) Drive to the tag on the far side
            actions.append(f"S{arrive_tag}")

            incoming_tag = arrive_tag

        return actions

    def run_navigation(self):
        """
        Main function to run the robot navigation:
        1) Reads the destination vertex.
        2) Receives the initial stop report S<tag> indicating the current location.
        3) Constructs the route and outputs the sequence of commands.
        4) Monitors execution, expecting exact reports for each command.
        """
        destination = "K"
        last_action = "S" + str(self.actual_tag)
        if not last_action.startswith("S"):
            print("Error: the first action must be a stop report 'S<tag>'")
            return

        tag_value = int(last_action[1:])
        edge, vertices = self.find_edge_by_tag(self.town_map, tag_value)
        if edge is None:
            print(f"No edge with tag {tag_value} was found!")
            return
        current_vertex = vertices[0]
        print(f"Detected current location: {current_vertex}")

        plan = self.find_shortest_path_instructions(self.town_map, current_vertex, destination)
        print("\nGenerated command sequence:")
        for cmd in plan:
            print(cmd)
        return ",".join(plan)


def main(args=None):
    rclpy.init(args=args)
    map_processor = MapNode()
    rclpy.spin(map_processor)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
