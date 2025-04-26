<a name="br1"></a> 

1-
```
STEERING_AXIS = 0
THROTTLE_AXIS = 1
```

=> Bu değişkenler, joystick verilerinin hangi eksende bulunduğunu belirtmek için

kullanılıyor.

\- `STEERING\_AXIS = 0`: Joystick verilerinde, direksiyon kontrolü için sol-sağ (x ekseni)

hareket kullanılır. Bu yüzden `STEERING\_AXIS` değişkeni x ekseni (sol-sağ hareket) için

0 olarak tanımlanmış.

\- `THROTTLE\_AXIS = 1`: Bu değişken y ekseni (ileri-geri hareket) için 1 olarak

tanımlanmış.

Bu değişkenler, joystick verilerini doğru eksene bağlama ve ilgili kontrolleri doğru

şekilde almak için kullanılır.

2-
```
class Translator:
    def __init__(self):
        self.sub = rospy.Subscriber("joy", Joy, self.callback)
        self.pub = rospy.Publisher('cart', cart_control, queue_size=1)
        self.last_published_time = rospy.get_rostime()
        self.last_published = None
        self.timer = rospy.Timer(rospy.Duration(1./20.), self.timer_callback)

```

`self.sub = rospy.Subscriber("joy", Joy, self.callback)`:

"joy" ROS topic'ine abone olunur. Bu topic, joystick verilerini yayınlayan bir

kaynaktan (bir joystick cihazı) gelen verileri alır. `Joy` türü, `sensor\_msgs.msg`

modülünden gelir ve joystick verilerini temsil eder. Sonra da veriler, `self.callback` adlı

bir geri çağırma fonksiyonuna iletilir.

`self.pub = rospy.Publisher('cart', cart\_control, queue\_size=1)`:

"cart" adlı bir ROS topic'ine mesaj yayınlamak için bir yayıncı oluşturulur.

`cart\_control` mesajı, `cart\_sim.msg` modülünden gelir ve aracın kontrol verilerini

temsil eder. `queue\_size=1`, yayıncının bekleyen mesajların maksimum sayısını

belirtir.

`self.last\_published\_time = rospy.get\_rostime()`:

ROS zamanını alır ve `self.last\_published\_time` değişkenine atar. Bu değişken,

son yayın zamanını takip etmek için kullanılır.

`self.last\_published = None`:

Bu satırda; değişken, son yayınlanan joystick verisini saklamak için kullanılır.

`self.timer = rospy.Timer(rospy.Duration(1./20.), self.timer\_callback)`:



<a name="br2"></a> 

Bu satırda, `rospy.Timer` fonksiyonu kullanılarak bir zamanlayıcı

oluşturulur.Burada, `rospy.Duration(1./20.)` ile 20 Hz'lik bir zamanlayıcı oluşturulur,

yani her 1/20 saniyede bir `self.timer\_callback` fonksiyonu çağrılır. Bu işlev, son

yayınlanan verinin ne zaman gönderildiğini kontrol etmek için kullanılır.

3-
```
def timer_callback(self, event):
        if self.last_published and self.last_published_time < rospy.get_rostime() + rospy.Duration(1.0/20.):
            self.callback(self.last_published)
```

`def timer\_callback(self, event):` :

Bu fonksiyon, bir zamanlayıcı tarafından çağrıldığında çalışır. `event` parametresi,

zamanlayıcı tarafından iletilen bir olay nesnesidir.

4-
```
def callback(self, message):
        rospy.logdebug("joy_translater received axes %s",message.axes)
        command = cart_control()
        command.header = message.header
        if message.axes[THROTTLE_AXIS] >= 0:
            command.throttle = message.axes[THROTTLE_AXIS]
            command.brake = 0.0
        else:
            command.brake = message.axes[THROTTLE_AXIS] * -1
            command.throttle = 0.0

        if message.buttons[3]:
            command.shift_gears = cart_control.FORWARD
        elif message.buttons[1]:
            command.shift_gears = cart_control.NEUTRAL
        elif message.buttons[0]:
            command.shift_gears = cart_control.REVERSE
        else:
            command.shift_gears = cart_control.NO_COMMAND

        command.steer = message.axes[STEERING_AXIS]
        self.last_published = message
        self.pub.publish(command)

```

`rospy.logdebug("joy\_translater received axes %s", message.axes)`:

Bu satır, bir hata ayıklama mesajı oluşturur ve "joy\_translater received axes" metnini ve alınan mesajdaki

eksende bulunan değerleri (`message.axes`) içerir. Bu mesaj, kodun doğru şekilde çalıştığını doğrulamak için

kullanılır.

`command = cart\_control()`:



<a name="br3"></a> 

Bu satırda, `cart\_control` türünden bir nesne oluşturulur. Bu nesne, aracın kontrol verilerini temsil eder.

`command.header = message.header`:

Bu satırda, `command` nesnesinin başlık (header) alanı, `message`'dan gelen başlıkla (`message.header`)

eşitlenir. Bu, kontrol komutlarının hangi zaman diliminde yayınlandığını takip etmek için kullanılır.

`if message.axes[THROTTLE\_AXIS] >= 0:`:

Joystick'ten alınan gaz kontrolü değerinin (`message.axes[THROTTLE\_AXIS]`) 0 veya daha büyük olduğunu

kontrol eder. Eğer bu değer 0 veya pozitifse, araç ileri hareket ediyordur ve ileri gaz kontrolü yapılır.

`command.throttle = message.axes[THROTTLE\_AXIS]`:

Eğer gaz kontrolü 0 veya pozitifse, aracın hızını belirlemek için `command` nesnesinin `throttle` alanı,

joystick'ten gelen gaz kontrolü değeriyle (`message.axes[THROTTLE\_AXIS]`) eşitlenir. Yani, aracın ileri hareket etmesi

sağlanır.

`command.brake = 0.0`:

Eğer gaz kontrolü 0 veya pozitifse, fren uygulanmaz ve `command` nesnesinin `brake` alanı sıfırlanır.

`command.brake = message.axes[THROTTLE\_AXIS] \* -1`:

Eğer gaz kontrolü 0'dan küçükse, fren uygulanır. Bu satırda, `command` nesnesinin `brake` alanı, joystick'ten

gelen geri gaz kontrolü değerinin negatif değeriyle (`message.axes[THROTTLE\_AXIS] \* -1`) eşitlenir. Bu, aracın geri

hareket etmesini sağlar.

`command.throttle = 0.0`:

Eğer gaz kontrolü 0'dan küçükse, gaz verilmez ve `command` nesnesinin `throttle`

alanı sıfırlanır.

`if message.buttons[3]:`, `elif message.buttons[1]:`, `elif message.buttons[0]:`, `else:`:

Bu satırlar, bir dizi koşul ifadesi oluşturur. Bu ifadeler, joystick'ten alınan düğme

durumlarına (`message.buttons`) bağlı olarak aracın vitesini belirler.

`command.steer = message.axes[STEERING\_AXIS]`:

Bu satırda, aracın direksiyon kontrolü belirlenir. `command` nesnesinin `steer` alanı,

joystick'ten alınan direksiyon kontrolü değeriyle (`message.axes[STEERING\_AXIS]`) eşitlenir.

Bu, aracın direksiyonunu belirler.

`self.last\_published = message`:

Bu satırda, `self.last\_published` değişkeni, işlenen mesajla (`message`) eşitlenir. Bu,

en son işlenen verinin takibi için kullanılır.

`self.pub.publish(command)`: Son olarak, `self.pub` yayıncısı kullanılarak `command`

nesnesi yayınlanır. Bu, araç kontrol komutlarının ROS topic'ine gönderilmesini sağlar ve

fiziksel bir araç üzerinde gerçek hareketi tetikler.



<a name="br4"></a> 

5-
```
if __name__ == '__main__':
    rospy.init_node('joy_translator')
    t = Translator()
    rospy.spin()
```

Bu kod, bir ROS düğümünü başlatır ve joystick verilerini araç kontrol komutlarına çeviren `Translator` sınıfını başlatır.

İşte her satırın açıklaması:

`rospy.init\_node('joy\_translator')`:

Bu, ROS üzerinde diğer düğümlerle iletişim kurmak için bir ana nokta sağlar.

`t = Translator()`:

Bu satırda, `Translator` sınıfından bir örnek oluşturulur. Bu, joystick verilerini alacak ve araç kontrol

komutlarına çevirecek olan çevirmen sınıfını başlatır.

`rospy.spin()`:

Bu satır, ROS düğümünün sonsuza kadar çalışmasını sağlar. Bu, ROS topic'lerinden veri almak ve yayınlamak

için gerekli döngüyü sağlar.
