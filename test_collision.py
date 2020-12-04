import collision

V = collision.Vector
C = collision.Circle

circle1 = C(V(0,0), 20)
circle2 = C(V(30,0), 20)
response = collision.Response()
collided = collision.test_circle_circle(circle1, circle2, response)

print("response.overlap:", response.overlap)
print("response.overlap:", response.overlap_v)