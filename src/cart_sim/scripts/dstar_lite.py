#!/usr/bin/env python3
import rospy
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from cart_sim.msg import cart_control
import heapq

class DStarLite:
    def __init__(self):
        # ROS düğümünü başlat
        rospy.init_node('dstar_lite')
        # Harita verilerini almak için bir abonelik oluştur
        self.grid_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        # Golf arabası kontrol komutlarını yayınlamak için bir yayıncı oluştur
        self.cmd_pub = rospy.Publisher("/cart", cart_control, queue_size=10)
        # Planlanan yolu yayınlamak için bir yayıncı oluştur
        self.path_pub = rospy.Publisher("/planned_path", Path, queue_size=10)
        # Başlangıç ve hedef noktalarını belirle
        self.start = (4, 620047, -3, 700560)  # Başlangıç noktası (X, Y koordinatları)
        self.goal = (45, 25)  # Hedef noktası (X, Y koordinatları)
        # Harita verilerini tutacak değişken
        self.grid = None
        # D* Lite algoritmasının öncelik kuyruğu ve km değişkeni
        self.U = []
        self.km = 0

    def map_callback(self, data):
        # Harita verilerini alır ve 2 boyutlu bir diziye dönüştürür
        width = data.info.width
        height = data.info.height
        self.grid = [[0 for _ in range(width)] for _ in range(height)]
        for i in range(height):
            for j in range(width):
                self.grid[i][j] = data.data[i * width + j]
        # Harita verisi yüklendikten sonra yol planlama fonksiyonunu çağır
        if self.grid is not None:
            self.plan_path()

    def heuristic(self, a, b):
        # Heuristic fonksiyonu: Manhattan mesafesi
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def calculate_key(self, node):
        # Düğümün anahtarını hesapla
        g_rhs = min(self.g[node], self.rhs[node])
        return (g_rhs + self.heuristic(self.start, node) + self.km, g_rhs)

    def update_vertex(self, node):
        # Bir düğümün durumunu günceller ve gerekli ise öncelik kuyruğuna ekler veya çıkarır.
        if self.g[node] != self.rhs[node]:
            heapq.heappush(self.U, (self.calculate_key(node), node))
        else:
            self.U = [item for item in self.U if item[1] != node]

    def compute_shortest_path(self):
        # En kısa yolu hesapla
        while self.U and (self.U[0][0] < self.calculate_key(self.start) or self.rhs[self.start] != self.g[self.start]):
            k_old, u = heapq.heappop(self.U)
            k_new = self.calculate_key(u)
            if k_old < k_new:
                heapq.heappush(self.U, (k_new, u))
            elif self.g[u] > self.rhs[u]:
                self.g[u] = self.rhs[u]
                for s in self.get_neighbors(u):
                    if s != self.goal:
                        self.rhs[s] = min(self.rhs[s], self.g[u] + 1)
                    self.update_vertex(s)
            else:
                g_old = self.g[u]
                self.g[u] = float('inf')
                for s in self.get_neighbors(u) + [u]:
                    if self.rhs[s] == g_old + 1:
                        if s != self.goal:
                            self.rhs[s] = min(self.get_cost(s) + self.g[s_prime] for s_prime in self.get_neighbors(s))
                    self.update_vertex(s)

    def get_neighbors(self, node):
        # Komşu düğümleri al
        neighbors = [(node[0] + dx, node[1] + dy) for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]]
        return [n for n in neighbors if 0 <= n[0] < len(self.grid) and 0 <= n[1] < len(self.grid[0]) and self.grid[n[0]][n[1]] == 0]

    def get_cost(self, node):
        # Bir düğümün geçiş maliyetini döndürür (bu örnekte sabit bir maliyet).
        return 1

    def plan_path(self):
        # Yolu planla
        if self.grid is None:
            return

        self.g = {node: float('inf') for row in self.grid for node in enumerate(row)}
        self.rhs = {node: float('inf') for row in self.grid for node in enumerate(row)}
        self.g[self.goal] = 0
        self.rhs[self.goal] = 0
        self.U = []
        self.km = 0

        heapq.heappush(self.U, (self.calculate_key(self.goal), self.goal))
        self.compute_shortest_path()

        path = []
        node = self.start
        while node != self.goal:
            path.append(node)
            node = min(self.get_neighbors(node), key=lambda n: self.g[n] + 1)
        path.append(self.goal)

        # Golf arabası kontrol mesajını yayınla
        control_msg = cart_control()
        control_msg.header.stamp = rospy.Time.now()
        control_msg.throttle = 1.0  # Gaz kontrolü
        control_msg.brake = 0.0  # Fren kontrolü
        control_msg.steer = 0.0  # Direksiyon kontrolü
        control_msg.shift_gears = cart_control.FORWARD  # Vites ayarı
        self.cmd_pub.publish(control_msg)

        # Path mesajını yayınla
        ros_path = Path()
        ros_path.header.frame_id = "map"
        for (x, y) in path:
            pose = PoseStamped()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            ros_path.poses.append(pose)
        self.path_pub.publish(ros_path)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    dstar = DStarLite()
    dstar.run()

# Bu kod, ROS ortamında çalışarak, harita verilerini alır ve D* Lite algoritması ile başlangıç ve hedef noktaları arasında bir yol planlar. 
# Planlanan yolu bir Path mesajı olarak yayınlar ve golf arabası için kontrol komutları gönderir. 
# D* Lite algoritması, dinamik ortamlarda en kısa yolu bulmak için etkili bir yöntemdir ve bu kodda, harita güncellemelerini de dikkate alarak yol planlaması yapar.
