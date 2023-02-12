# Example ontology
Robot sees sequences of small integer arrays (1-5 elements), which represent object ids from small set (1-25).

It listens ROS Message Queue and write to short-term memory:
```
CREATE (e1:Event {timestamp: 1})
MERGE (p1:Object {id: 1})
CREATE (e1) -[:I_SEE]-> (p1)
MERGE (p2:Object {id: 2})
CREATE (e1) -[:I_SEE]-> (p2)


CREATE (e2:Event {timestamp: 2})
MERGE (p1:Object {id: 1})
CREATE (e1) -[:I_SEE]-> (p1)
MERGE (e1:Event {timestamp: 1})
CREATE (e1) -[:NEXT_EVENT]-> (e2)
```

It can run short-term memory query:
```
// Seen recently
MATCH (p:Object {id: 1})
MATCH (p) <-[:I_SEE]- () <-[:NEXT_EVENT *1..5]- (prev_e) -[:I_SEE]-> (p)
RETURN count(prev_e) AS result
```

It can run analytical query (in charge time) and translate short-trem memory to long-term:
```
// TODO Query: Recall who appeared toghether 3 or more times and make relationships -[:KNOWS]-
```

It can run graph algorithm and infer new knowledge in long-term memory:
```
// Memgraph MAGE
// TODO Query: Run community detection on -[:KNOWS]- and write "community_id" feature for :Object node
```


# Example code
## event_graph_writer
ROS Message Queue subscriber, write stream data to Memgraph
```
ros2 run ros_memgraph_examples_event_graph_writer event_graph_writer
```
## event_publisher
test-purpose events generator
```
ros2 run ros_memgraph_examples_event_publisher event_publisher
```
## queries/seen_recently
ROS Service, example analytical query
## queries/community_detection
Example graph algorithm usage (MAGE cugraph.louvain)


## Library development
https://github.com/Graph-Power-Solutions/warehouse_ros_graph