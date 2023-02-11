CREATE (p1:Object {id: 1})
CREATE (p1:Object {id: 2})
CREATE (p1:Object {id: 3})

CREATE (e1:Event {timestamp: 1}) -[:I_SEE]-> (p1)

CREATE (e2:Event {timestamp: 2}) -[:I_SEE]-> (p2), (e1) -[:NEXT_EVENT]-> (e2)

CREATE (e3:Event {timestamp: 3}) -[:I_SEE]-> (p1), (e2) -[:NEXT_EVENT]-> (e3)

CREATE (e4:Event {timestamp: 4}) -[:I_SEE]-> (p3), (e3) -[:NEXT_EVENT]-> (e4)


