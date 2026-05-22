/// Parametre metadata sözlüğü — param_tuner_dialog.dart tarafından kullanılır.
///
/// Her node için, ROS2 parametre adından insan-okuyabilir bilgilere bir
/// eşleştirme. Açıklamalar Türkçe ve tuning rehberi sunar: bu parametrenin
/// ne yaptığı, ne zaman değiştirilmesi gerektiği, hangi yöne hangi etki.
///
/// Yeni parametre eklemek istersen: nodu bul, ParamMeta ekle, gerekirse
/// param_tuner_dialog.dart içindeki _quickParams listesine de ekle.

class ParamMeta {
  /// Türkçe kısa etiket (paramın ne işe yaradığının özeti, 1-3 kelime).
  final String label;

  /// Uzun açıklama: bu parametre ne yapıyor, hangi sistemde rol oynuyor.
  /// 1-3 cümle, Türkçe.
  final String description;

  /// Tuning ipucu: parametreyi artırırsan/azaltırsan ne olur.
  /// Genelde "↑ X, ↓ Y" formatında.
  final String? effect;

  /// Birim (örn. "m/s", "rad", "m", "µs", "rad/s²").
  final String? unit;

  /// Tipik kullanılabilir aralık (alt-üst sınır), guidance amaçlı.
  /// Sabit teknik limit değil — sadece tahmini bir aralık.
  final double? typicalMin;
  final double? typicalMax;

  /// Kategori — params'ı grupla. Örn. "Hız", "Carrot", "Goal Toleransı".
  final String category;

  const ParamMeta({
    required this.label,
    required this.description,
    required this.category,
    this.effect,
    this.unit,
    this.typicalMin,
    this.typicalMax,
  });

  bool get hasRange => typicalMin != null && typicalMax != null;
}

/// node path → param name → metadata.
/// Param adları ROS2 ListParameters'ın döndürdüğü ile birebir eşleşmeli
/// (örn. "FollowPath.desired_linear_vel", "inflation_layer.inflation_radius").
const paramMetadata = <String, Map<String, ParamMeta>>{
  // ════════════════════════════════════════════════════════════════════
  // /nav2_pico_bridge — RPi5 ↔ Pico ASCII serial köprüsü
  // ════════════════════════════════════════════════════════════════════
  '/nav2_pico_bridge': {
    'vx_polarity': ParamMeta(
      label: 'İleri/Geri yön',
      description:
          'Aracın ileri yönünü düzeltir. +1 standart: ROS-pozitif vx aracı ileri sürer. '
          '-1: motor kabloları/şase ters montajlıysa veya pozitif vx aracı geri götürüyorsa.',
      effect: 'Yanlışsa: ileri komutu geri gidiyor (veya tersi)',
      category: 'Yön Kalibrasyonu',
      typicalMin: -1, typicalMax: 1,
    ),
    'servo_polarity': ParamMeta(
      label: 'Direksiyon yönü',
      description:
          'Direksiyon (servo) yönünü düzeltir. Sol komutu (pozitif wz) aracı sağa çeviriyorsa '
          'bu değeri -1 yap. Linkage mekanik geometrisine bağlı.',
      effect: 'Yanlışsa: sol komut sağa dönüş yapar',
      category: 'Yön Kalibrasyonu',
      typicalMin: -1, typicalMax: 1,
    ),
    'reverse_steer_polarity': ParamMeta(
      label: 'Geri vites direksiyon yönü',
      description:
          'Sadece geri viteste direksiyon yönünü çevirir. REEDS_SHEPP planner geriye-ileri '
          'cusp\'larda direksiyon hissini korumak için kullanılır. Forward yön doğruysa, '
          'reverse\'de tekerlekler ters tarafa bakıyorsa -1 yap.',
      category: 'Yön Kalibrasyonu',
      typicalMin: -1, typicalMax: 1,
    ),
    'max_vx_mps': ParamMeta(
      label: 'Max ileri hız',
      description:
          'Bridge\'in kabul edeceği max ileri hız. Bunun üstündeki Nav2 komutları kırpılır. '
          'Nav2\'nin desired_linear_vel\'i bunun altında olmalı (yoksa bridge kesip atar).',
      effect: '↑ daha hızlı ama precision azalır',
      unit: 'm/s',
      category: 'Hız Limitleri',
      typicalMin: 0.10, typicalMax: 0.50,
    ),
    'max_wz_radps': ParamMeta(
      label: 'Max açısal hız',
      description:
          'Max yaw rate limiti. Çok yüksek değer servo\'nun fiziksel yapamadığı dönüş '
          'isteğine yol açar → "thunk" sesi + stale steering.',
      effect: '↑ daha keskin dönüş, ↓ yumuşak dönüş',
      unit: 'rad/s',
      category: 'Hız Limitleri',
      typicalMin: 0.30, typicalMax: 1.50,
    ),
    'max_ax_mps2': ParamMeta(
      label: 'Lineer ivme cap\'i',
      description:
          'Bridge\'in tek tick\'te uygulayabileceği max lineer ivme. Velocity smoother\'dan '
          'gelen aggressive komutlara karşı son katman cap.',
      effect: '↓ yumuşak ivmelenme (snake azalır), ↑ hızlı tepki',
      unit: 'm/s²',
      category: 'İvme Limitleri',
      typicalMin: 0.20, typicalMax: 1.50,
    ),
    'max_aw_radps2': ParamMeta(
      label: 'Açısal ivme cap\'i',
      description:
          'Tek tick\'te uygulanabilen max yaw ivmesi. Goal etrafında osilasyonu ve servo '
          'jitter\'ını azaltır.',
      effect: '↓ yumuşak dönüş başlangıcı (oscillation azalır)',
      unit: 'rad/s²',
      category: 'İvme Limitleri',
      typicalMin: 0.30, typicalMax: 2.00,
    ),
    'max_steer_rate_radps': ParamMeta(
      label: 'Servo slew rate',
      description:
          'Servo açısının değişim hızı sınırı. MG995 için ~2.0-2.7 rad/s gerçekçi. '
          'Çok yüksek olursa servo isteği yetiştiremez, çok düşük olursa direksiyon tepkisi gecikir.',
      effect: '↓ yumuşak direksiyon, ↑ keskin (servo zorlanabilir)',
      unit: 'rad/s',
      category: 'Direksiyon',
      typicalMin: 1.0, typicalMax: 4.0,
    ),
    'min_vx_creep': ParamMeta(
      label: 'Min creep hızı',
      description:
          'Bu mutlak hızın altındaki vx komutları 0\'a clip\'lenir. L298N motor sürücüsünün '
          'static-friction deadband\'ini aşamayan komutların gereksiz titreşim yapmasını önler. '
          'Goal yaklaşımında micro-lurching\'i kaldırır.',
      effect: '↑ goal\'a daha temiz oturma, ↓ daha düşük hızda creep mümkün',
      unit: 'm/s',
      category: 'Deadband',
      typicalMin: 0.0, typicalMax: 0.10,
    ),
    'servo_center_us': ParamMeta(
      label: 'Servo merkez PWM',
      description:
          'Tekerleklerin fiziksel olarak DÜZ durduğu PWM değeri. Kalibrasyon parametresi — '
          'her şasiye özgüdür. Settings → Calibrate Direction wizard\'ı bunu otomatik bulur.',
      effect: 'Yanlışsa: düz path\'te araç sürekli sola/sağa çekiyor',
      unit: 'µs',
      category: 'Servo PWM',
      typicalMin: 1400, typicalMax: 1700,
    ),
    'servo_us_min': ParamMeta(
      label: 'Sol direksiyon PWM limit',
      description:
          'Bridge\'in sola izin verdiği min servo PWM. Pico\'nun mekanik limit (500 µs) ile '
          'merkez (servo_center_us) arasında olmalı. Düşür → daha geniş sol dönüş açısı '
          '(turning radius küçülür) ama linkage zorlanabilir.',
      effect: '↓ daha tight sol dönüş, ↑ daha geniş ama korunaklı',
      unit: 'µs',
      category: 'Servo PWM',
      typicalMin: 1100, typicalMax: 1500,
    ),
    'servo_us_max': ParamMeta(
      label: 'Sağ direksiyon PWM limit',
      description:
          'Bridge\'in sağa izin verdiği max servo PWM. servo_center ile mekanik limit (2500 µs) '
          'arasında. Arttır → daha geniş sağ dönüş açısı ama linkage zorlanabilir.',
      effect: '↑ daha tight sağ dönüş, ↓ daha geniş ama korunaklı',
      unit: 'µs',
      category: 'Servo PWM',
      typicalMin: 1800, typicalMax: 2200,
    ),
  },

  // ════════════════════════════════════════════════════════════════════
  // /controller_server — MPPI (varsayılan) veya RPP (fallback) + Goal Checker
  // MPPI params yalnızca controller:=mppi ile, RPP params yalnızca
  // controller:=rpp ile görünür (tuner canlı ListParameters'a göre filtreler).
  // ════════════════════════════════════════════════════════════════════
  '/controller_server': {
    // ── MPPI — Hız / Limit ──────────────────────────────────────────
    'FollowPath.vx_max': ParamMeta(
      label: 'MPPI max ileri hız',
      description:
          'MPPI\'nin örneklediği trajektörilerin üst hız sınırı. velocity_smoother '
          'max_velocity[0] ile aynı tutulmalı. Settings → Nav2 Max Speed slider\'ı MPPI '
          'modunda bu parametreyi değiştirir.',
      effect: '↑ hızlı sürüş, ↓ precision',
      unit: 'm/s',
      category: 'MPPI Hız/Limit',
      typicalMin: 0.05, typicalMax: 0.40,
    ),
    'FollowPath.vx_min': ParamMeta(
      label: 'MPPI max geri hız',
      description:
          'Negatif değer geri gitmeyi açar (K-dönüş / geri park). 0 yaparsan MPPI '
          'yalnızca ileri sürer.',
      effect: '↓ (daha negatif) daha hızlı geri manevra',
      unit: 'm/s',
      category: 'MPPI Hız/Limit',
      typicalMin: -0.30, typicalMax: 0.0,
    ),
    'FollowPath.wz_max': ParamMeta(
      label: 'MPPI max dönüş hızı',
      description:
          'Örneklenen maksimum açısal hız. velocity_smoother wz cap (0.5) ile uyumlu tut.',
      unit: 'rad/s',
      category: 'MPPI Hız/Limit',
      typicalMin: 0.2, typicalMax: 1.0,
    ),
    'FollowPath.AckermannConstraints.min_turning_r': ParamMeta(
      label: 'Min dönüş yarıçapı',
      description:
          'MPPI\'nin uyacağı minimum dönüş yarıçapı. Planner minimum_turning_radius '
          '(0.50) ile aynı tutulmalı — yoksa controller planner\'ın çizemeyeceği yolları dener.',
      unit: 'm',
      category: 'MPPI Hız/Limit',
      typicalMin: 0.40, typicalMax: 0.80,
    ),
    // ── MPPI — Optimizer ────────────────────────────────────────────
    'FollowPath.batch_size': ParamMeta(
      label: 'Trajektöri sayısı',
      description:
          'Her tick\'te örneklenen trajektöri sayısı. ASIL Pi 5 CPU kolu — /cmd_vel Hz '
          'düşerse ilk bunu düşür (1000 → 800 → 600).',
      effect: '↑ daha iyi çözüm + çok CPU, ↓ hafif ama kaba',
      category: 'MPPI Optimizer',
      typicalMin: 400, typicalMax: 2000,
    ),
    'FollowPath.time_steps': ParamMeta(
      label: 'Horizon adım sayısı',
      description:
          'Trajektöri horizon uzunluğu (adım). horizon_s = time_steps × model_dt. '
          'Fazla → daha smooth ama daha çok CPU. Pi 5 back-off: 40 → 30 → 24.',
      effect: '↑ smooth/uzak görüş + CPU, ↓ hafif ama kısa görüş',
      category: 'MPPI Optimizer',
      typicalMin: 20, typicalMax: 60,
    ),
    'FollowPath.model_dt': ParamMeta(
      label: 'Model zaman adımı',
      description:
          'Her örnekleme adımının süresi. 1/controller_frequency\'ye eşit olmalı (10 Hz → 0.1).',
      unit: 's',
      category: 'MPPI Optimizer',
      typicalMin: 0.05, typicalMax: 0.2,
    ),
    'FollowPath.temperature': ParamMeta(
      label: 'Softmax sıcaklığı',
      description:
          'Trajektöri ağırlıklandırmasının seçiciliği. Düşük → daha açgözlü (en iyi '
          'trajektöriye yapışır), yüksek → daha yumuşak ortalama.',
      category: 'MPPI Optimizer',
      typicalMin: 0.1, typicalMax: 0.5,
    ),
    'FollowPath.vx_std': ParamMeta(
      label: 'Hız örnekleme std',
      description:
          'Örneklenen lineer hız gürültüsünün std-sapması. Yüksek → daha keşifçi '
          'trajektöriler ama daha çok CPU.',
      effect: '↑ daha çeşitli trajektöri + CPU, ↓ daha dar arama',
      unit: 'm/s',
      category: 'MPPI Optimizer',
      typicalMin: 0.05, typicalMax: 0.3,
    ),
    'FollowPath.wz_std': ParamMeta(
      label: 'Dönüş örnekleme std',
      description:
          'Örneklenen açısal hız gürültüsünün std-sapması. Yüksek → daha agresif dönüş '
          'arama ama daha çok CPU.',
      unit: 'rad/s',
      category: 'MPPI Optimizer',
      typicalMin: 0.1, typicalMax: 0.6,
    ),
    // ── MPPI — Critic ağırlıkları ───────────────────────────────────
    'FollowPath.CostCritic.cost_weight': ParamMeta(
      label: 'Engel kaçınma ağırlığı',
      description:
          'CostCritic = engel/duvar kaçınma. collision_monitor kapalı olduğundan ASIL '
          'engel güvenliği bu. ↑ duvarlardan daha çok kaçınır.',
      effect: '↑ daha geniş duvar payı, ↓ duvarlara daha yakın geçer',
      category: 'MPPI Critic',
      typicalMin: 1.0, typicalMax: 8.0,
    ),
    'FollowPath.PathAlignCritic.cost_weight': ParamMeta(
      label: 'Path\'e hizalanma ağırlığı',
      description:
          'Trajektörinin planlanan path\'e yanal hizalanmasını ödüllendirir. ↑ path\'i '
          'sıkı takip (lane merkezi), ↓ daha serbest sapma.',
      category: 'MPPI Critic',
      typicalMin: 2.0, typicalMax: 20.0,
    ),
    'FollowPath.PathFollowCritic.cost_weight': ParamMeta(
      label: 'Path ilerleme ağırlığı',
      description:
          'Path boyunca ileri ilerlemeyi ödüllendirir. Düşükse robot yerinde oyalanabilir.',
      category: 'MPPI Critic',
      typicalMin: 2.0, typicalMax: 10.0,
    ),
    'FollowPath.GoalCritic.cost_weight': ParamMeta(
      label: 'Hedefe ulaşma ağırlığı',
      description: 'Hedef xy konumuna yakınsamayı ödüllendirir (yaklaşma bölgesinde).',
      category: 'MPPI Critic',
      typicalMin: 2.0, typicalMax: 10.0,
    ),
    'FollowPath.PreferForwardCritic.cost_weight': ParamMeta(
      label: 'İleri tercih ağırlığı',
      description:
          'İleri sürüşü tercih ettirir; gereksiz geri manevraları azaltır (vx_min<0 ile '
          'gerçek geri park hâlâ mümkün).',
      category: 'MPPI Critic',
      typicalMin: 1.0, typicalMax: 10.0,
    ),
    'FollowPath.PathAngleCritic.cost_weight': ParamMeta(
      label: 'Yön (heading) ağırlığı',
      description:
          'Trajektöri yönünün path yönüyle uyumunu ödüllendirir. mode:2 ileri+geri '
          'yönleri birlikte değerlendirir (reverse cusp\'lar engellenmez).',
      category: 'MPPI Critic',
      typicalMin: 1.0, typicalMax: 6.0,
    ),

    // ── RPP (fallback) — Hız ────────────────────────────────────────
    'FollowPath.desired_linear_vel': ParamMeta(
      label: 'İstenen cruise hızı',
      description:
          'RPP\'nin düz path\'te ulaşmaya çalıştığı hedef hız. Settings → Nav2 Max Speed '
          'slider\'ı bunu live olarak değiştirir. Park robotu için 0.10-0.15 m/s precision\'a uygun, '
          '0.20+ açık alan için.',
      effect: '↑ hızlı sürüş, ↓ precision (snake azalır)',
      unit: 'm/s',
      category: 'Hız',
      typicalMin: 0.05, typicalMax: 0.40,
    ),
    'FollowPath.regulated_linear_scaling_min_speed': ParamMeta(
      label: 'Tight curve min hız',
      description:
          'Regulator dar curve\'lerde hızı bu değere kadar düşürebilir. Bridge\'in min_vx_creep\'inin '
          'üstünde olmalı (yoksa motor hiç dönmez). Düşük tut → curve\'lerde rahat yavaşlasın.',
      effect: '↓ curve\'lerde rahat yavaşlama, ↑ minimum hızı yüksek tut',
      unit: 'm/s',
      category: 'Hız',
      typicalMin: 0.02, typicalMax: 0.20,
    ),
    'FollowPath.max_vel_x': ParamMeta(
      label: 'Max linear hız (legacy DWB)',
      description:
          'DWB için kullanılır. RPP yüklü olduğunda no-op. Eski tuning script\'leri ile '
          'uyumluluk için kalıyor.',
      unit: 'm/s',
      category: 'Hız',
    ),

    // ── Lookahead Carrot ─────────────────────────────────────────────
    'FollowPath.lookahead_dist': ParamMeta(
      label: 'Carrot mesafesi',
      description:
          'RPP\'nin path üzerinde "şu noktayı takip et" diye seçtiği carrot mesafesi. '
          'use_velocity_scaled_lookahead_dist=true ise hıza göre min/max arasında interpolate edilir. '
          'Küçük → tight tracking ama oscillation; büyük → smooth ama corner cut.',
      effect: '↑ smooth ama köşe yumuşar, ↓ tight ama snake',
      unit: 'm',
      category: 'Carrot / Lookahead',
      typicalMin: 0.15, typicalMax: 0.50,
    ),
    'FollowPath.min_lookahead_dist': ParamMeta(
      label: 'Min carrot mesafesi',
      description:
          'Düşük hızda bile carrot bu mesafeden yakına inmez. Robot footprint\'inin yarı-uzunluğundan '
          'BÜYÜK olmalı (footprint yarısı 0.15m → en az 0.18-0.20). Yoksa carrot kendi içinde kalır → snake.',
      effect: '↑ snake azalır, ↓ daha tight tracking ama oscillation',
      unit: 'm',
      category: 'Carrot / Lookahead',
      typicalMin: 0.15, typicalMax: 0.30,
    ),
    'FollowPath.max_lookahead_dist': ParamMeta(
      label: 'Max carrot mesafesi',
      description:
          'Yüksek hızda bile carrot bu mesafeden uzağa çıkmaz. Çok büyük → araç kısa path\'lerde '
          'carrot\'ı path sonuna fırlatır, corner cut yapar.',
      effect: '↑ uzun düz path\'te smooth, ↓ goal yakınında tight',
      unit: 'm',
      category: 'Carrot / Lookahead',
      typicalMin: 0.25, typicalMax: 0.60,
    ),
    'FollowPath.lookahead_time': ParamMeta(
      label: 'Velocity-scaled lookahead süresi',
      description:
          'use_velocity_scaled_lookahead_dist=true iken, carrot = vx × bu süre (min/max ile clamp). '
          'Yüksek (1.0+) → yüksek hızda carrot çok ileride → sharp turn miss. Düşük (0.4-0.6) → tight tracking.',
      effect: '↓ tight tracking, ↑ smooth ama tepki gecikir',
      unit: 's',
      category: 'Carrot / Lookahead',
      typicalMin: 0.3, typicalMax: 1.2,
    ),
    'FollowPath.curvature_lookahead_dist': ParamMeta(
      label: 'Curvature carrot mesafesi',
      description:
          'Direksiyon komutu hesabı için AYRI carrot. Steering smooth\'luğunu kontrol eder. '
          'Genelde lookahead_dist ile aynı veya yakın değer tutulur.',
      effect: '↑ smooth direksiyon, ↓ keskin response',
      unit: 'm',
      category: 'Carrot / Lookahead',
      typicalMin: 0.15, typicalMax: 0.30,
    ),

    // ── Eğri & Cost Yavaşlatma ───────────────────────────────────────
    'FollowPath.regulated_linear_scaling_min_radius': ParamMeta(
      label: 'Curve yavaşlama eşiği',
      description:
          'Path\'in curvature yarıçapı bu değerden küçükse RPP hızı azaltır. Planner\'ın '
          'minimum_turning_radius\'undan büyük olmalı (~1.4× iyi bir çarpan).',
      effect: '↑ tight curve\'lerde daha agresif yavaşlama, ↓ sabit hız',
      unit: 'm',
      category: 'Eğri / Cost Yavaşlatma',
      typicalMin: 0.40, typicalMax: 2.00,
    ),
    'FollowPath.cost_scaling_dist': ParamMeta(
      label: 'Cost yavaşlama mesafesi',
      description:
          'Engel/duvar inflation\'ına bu mesafeden itibaren tepki vermeye başlar. '
          'Yüksek → uzak engelden yavaşlar (overly cautious). Düşük → engele yakın geçer (cesur).',
      effect: '↑ engellere karşı erken yavaşla, ↓ cesur sürüş',
      unit: 'm',
      category: 'Eğri / Cost Yavaşlatma',
      typicalMin: 0.30, typicalMax: 1.00,
    ),
    'FollowPath.cost_scaling_gain': ParamMeta(
      label: 'Cost yavaşlama şiddeti',
      description:
          'Costmap inflation yakınlığına ne kadar agresif yavaşlanacağı (0-1 arası). '
          'Yüksek (~1.0) → wall yakını snake-stop. Düşük (0.5-0.7) → smooth geçiş.',
      effect: '↓ smooth (snake azalır), ↑ wall\'lardan çok kaçınma',
      category: 'Eğri / Cost Yavaşlatma',
      typicalMin: 0.3, typicalMax: 1.0,
    ),

    // ── Collision Detection (RPP-içi) ────────────────────────────────
    'FollowPath.max_allowed_time_to_collision_up_to_carrot': ParamMeta(
      label: 'RPP collision predict süresi',
      description:
          'RPP path boyunca bu kadar saniye ileriye forward simulate eder, collision görürse '
          'hızı SIFIRLAR. Yüksek (>1.0s) → duvar yanında "plan var hareket yok" sendromu. '
          'Düşük (0.5-0.7s) → daha cesur ama gerçek tehlike algılansın diye yine de aktif kalsın.',
      effect: '↓ duvar yanında stall azalır, ↑ daha defensif',
      unit: 's',
      category: 'Collision',
      typicalMin: 0.3, typicalMax: 1.5,
    ),

    // ── Ackermann ────────────────────────────────────────────────────
    'FollowPath.use_rotate_to_heading': ParamMeta(
      label: 'In-place rotation',
      description:
          'Goal yaw\'ı düzeltirken yerinde dönmeye izin verir. ACKERMANN ROBOT YERİNDE DÖNEMEZ! '
          'Bu her zaman FALSE olmalı, yoksa controller stall eder.',
      effect: 'TRUE: controller stall (Ackermann için imkansız)',
      category: 'Ackermann',
    ),
    'FollowPath.allow_reversing': ParamMeta(
      label: 'Geri vites kullanımı',
      description:
          'RPP\'nin geri-vites segmentlerini takip etmesine izin verir. REEDS_SHEPP planner ile '
          'kullanılmalı. 3-point park ve K-turn manevraları için MUTLAKA AÇIK olmalı.',
      effect: 'TRUE (gerekli) — kapatma',
      category: 'Ackermann',
    ),

    // ── Goal Tolerance ───────────────────────────────────────────────
    'general_goal_checker.xy_goal_tolerance': ParamMeta(
      label: 'XY goal toleransı',
      description:
          'Aracın goal pozisyonundan bu mesafeye kadar yaklaşması "ulaştı" sayılır. '
          'Çok düşük (<5cm) → araç hunting yapar, oturamaz. Park precision için 5-15cm makul.',
      effect: '↓ tighter precision ama hunting riski, ↑ rahat oturma',
      unit: 'm',
      category: 'Goal Tolerance',
      typicalMin: 0.05, typicalMax: 0.25,
    ),
    'general_goal_checker.yaw_goal_tolerance': ParamMeta(
      label: 'Yaw goal toleransı',
      description:
          'Aracın goal yaw\'ından bu kadar farklı olabilir, hâlâ "ulaştı" sayılır. '
          'Ackermann robot in-place dönemediğinden tight (<5°) genelde imkansız → oscillation. '
          '0.10-0.15 rad (~6-9°) pratik.',
      effect: '↓ tighter yaw ama oscillation, ↑ rahat oturma',
      unit: 'rad',
      category: 'Goal Tolerance',
      typicalMin: 0.05, typicalMax: 0.20,
    ),

    // ── Park Yaklaşımı ───────────────────────────────────────────────
    'FollowPath.approach_velocity_scaling_dist': ParamMeta(
      label: 'Goal yaklaşma yavaşlama mesafesi',
      description:
          'Goal\'e bu kadar yakın olunca RPP hızı min_approach_linear_velocity\'ye doğru lineer '
          'olarak düşürür. Park manevrasında smooth approach için kritik.',
      effect: '↑ daha erken yavaşla (park güzelleşir), ↓ son anda yavaşla',
      unit: 'm',
      category: 'Park Yaklaşımı',
      typicalMin: 0.20, typicalMax: 1.00,
    ),
    'FollowPath.min_approach_linear_velocity': ParamMeta(
      label: 'Yaklaşma min hızı',
      description:
          'Goal yakınında inilebilecek min hız. Bridge\'in min_vx_creep\'inin biraz üstünde '
          'olmalı (motor hareketsiz kalmasın).',
      effect: '↓ son metrede çok yavaş creep, ↑ goal\'a hızlı varış',
      unit: 'm/s',
      category: 'Park Yaklaşımı',
      typicalMin: 0.02, typicalMax: 0.10,
    ),
  },

  // ════════════════════════════════════════════════════════════════════
  // /planner_server — SMAC Hybrid-A* (GridBased)
  // ════════════════════════════════════════════════════════════════════
  '/planner_server': {
    'GridBased.minimum_turning_radius': ParamMeta(
      label: 'Min dönüş yarıçapı',
      description:
          'Planner bu yarıçaptan daha tight curve\'ler ÜRETMEZ. ARACIN FİZİKSEL R\'sinden BÜYÜK '
          'olmalı (servo limit + Ackermann linkage geometrisi ile belirlenir). '
          'AutoNexa MG995 + Hiwonder linkage ile yaklaşık 0.85-1.20m arası, conservative '
          'başlangıç 1.00m. Testte aşağı doğru iteratif azalt.',
      effect: '↓ daha tight curve\'ler (snake riski), ↑ geniş arc\'lar (testbed sığmaz)',
      unit: 'm',
      category: 'Geometry',
      typicalMin: 0.40, typicalMax: 1.50,
    ),
    'GridBased.reverse_penalty': ParamMeta(
      label: 'Geri vites cezası',
      description:
          'Geri-vites segmentlerinin maliyet çarpanı. Yüksek (≥2.0) → planner forward path\'i '
          'tercih eder. Düşük (1.0-1.5) → 3-point park kolayca emit edilir. allow_reversing aktif olmalı.',
      effect: '↓ daha kolay 3-point park, ↑ forward bias (cusp az)',
      category: 'Search Penalties',
      typicalMin: 1.0, typicalMax: 3.0,
    ),
    'GridBased.change_penalty': ParamMeta(
      label: 'Forward↔Reverse geçiş cezası',
      description:
          'Path içindeki direction switch (cusp) cost\'u. Yüksek → planner gereksiz cusp\'lardan kaçınır.',
      effect: '↑ az cusp (smooth path), ↓ planner daha fazla cusp atabilir',
      category: 'Search Penalties',
      typicalMin: 0.0, typicalMax: 1.0,
    ),
    'GridBased.non_straight_penalty': ParamMeta(
      label: 'Curve cezası',
      description:
          'Curve\'ler düz çizgilere karşı bu kadar pahalı. Yüksek (≥1.2) → planner düz path tercih eder. '
          'R_min wide olduğunda düşürmek (1.05) curve\'lere izin verir → engel etrafından dolaşma kolaylaşır.',
      effect: '↑ düz path bias, ↓ planner daha rahat curve eder',
      category: 'Search Penalties',
      typicalMin: 1.00, typicalMax: 1.50,
    ),
    'GridBased.cost_penalty': ParamMeta(
      label: 'Costmap cost cezası',
      description:
          'Costmap\'teki yüksek-cost hücrelerden (inflation, engel yakını) kaçınma şiddeti.',
      effect: '↑ engellerden daha çok kaçın, ↓ tight geçişlere izin ver',
      category: 'Search Penalties',
      typicalMin: 1.0, typicalMax: 3.0,
    ),
    'GridBased.analytic_expansion_ratio': ParamMeta(
      label: 'Analytic expansion oranı',
      description:
          'Planner heuristic\'i bu orana ulaşınca Reeds-Shepp ile direkt closing dener. '
          'Düşük (2.0-2.5) → final segment kısa ve temiz. Yüksek → planner daha çok grid expansion '
          'yapar ama final segment uzun olabilir.',
      effect: '↓ park manevrasında kısa final, ↑ daha kapsamlı arama',
      category: 'Search',
      typicalMin: 1.5, typicalMax: 5.0,
    ),
    'GridBased.max_planning_time': ParamMeta(
      label: 'Max planlama süresi',
      description:
          'Planner\'a verilen max süre. Aşılırsa fail. 2x2m testbed için 2.0s yeterli.',
      effect: '↑ karmaşık plan\'lara fırsat, ↓ daha hızlı fail',
      unit: 's',
      category: 'Search',
      typicalMin: 1.0, typicalMax: 5.0,
    ),
  },

  // ════════════════════════════════════════════════════════════════════
  // /velocity_smoother — Nav2 → bridge arası velocity smoothing
  // ════════════════════════════════════════════════════════════════════
  '/velocity_smoother': {
    'max_velocity': ParamMeta(
      label: 'Max hız vektörü [vx, vy, wz]',
      description:
          'Smoother\'ın izin verdiği max hızlar. Format: [lineer m/s, yan m/s (0), açısal rad/s]. '
          'vx controller\'ın desired_linear_vel\'i ile veya biraz üstünde olmalı.',
      effect: 'Liste — virgülle ayırarak gir',
      category: 'Hız Cap',
    ),
    'min_velocity': ParamMeta(
      label: 'Min hız vektörü',
      description:
          'Negatif değerler reverse\'e izin verir. allow_reversing ile uyumlu olmalı.',
      category: 'Hız Cap',
    ),
    'max_accel': ParamMeta(
      label: 'Max ivme vektörü',
      description:
          'Pozitif ivmelenme cap\'leri. Format: [vx m/s², vy (0), wz rad/s²]. '
          'Düşük değer → daha yumuşak hızlanma, snake azalır. Çok düşük → goal\'a yetişemez.',
      effect: 'Liste — düşük: yumuşak, yüksek: keskin',
      category: 'İvme',
    ),
    'max_decel': ParamMeta(
      label: 'Max fren vektörü',
      description:
          'Negatif olmalı! Yavaşlama cap\'leri. Çok agresif (-3.0) → goal yakınında servo jitter. '
          'Çok yumuşak → goal\'da overshoot.',
      effect: 'Negatif değerler — fren şiddeti',
      category: 'İvme',
    ),
    'smoothing_frequency': ParamMeta(
      label: 'Smoothing frekansı',
      description:
          'Smoother\'ın çalışma frekansı (Hz). Controller frequency (20Hz) ile uyumlu olmalı.',
      effect: '↑ daha tepki, ↓ CPU tasarrufu',
      unit: 'Hz',
      category: 'Sistem',
      typicalMin: 10, typicalMax: 30,
    ),
  },

  // ════════════════════════════════════════════════════════════════════
  // /global_costmap/global_costmap
  // ════════════════════════════════════════════════════════════════════
  '/global_costmap/global_costmap': {
    'inflation_layer.inflation_radius': ParamMeta(
      label: 'Inflation yarıçapı (global)',
      description:
          'Engel etrafına eklenen "kaçınılacak alan" yarıçapı. Yüksek → planner çok defensif, '
          'dar alanlardan geçemez. Düşük → engele yakın plan üretir (controller takip edemeyebilir). '
          'Robot footprint\'ten biraz büyük tut (5-10cm iyi).',
      effect: '↑ defensif planlama, ↓ tight geçiş',
      unit: 'm',
      category: 'Inflation',
      typicalMin: 0.02, typicalMax: 0.30,
    ),
    'inflation_layer.cost_scaling_factor': ParamMeta(
      label: 'Cost decay (global)',
      description:
          'Inflation cost\'unun engelden uzaklaştıkça ne kadar hızlı düştüğü. Yüksek → cost hızla '
          'düşer (gradient sharp). Düşük → uzun mesafeye yayılır (smooth).',
      effect: '↑ cost gradient sharp, ↓ smooth ama uzun yayılım',
      category: 'Inflation',
      typicalMin: 1.0, typicalMax: 10.0,
    ),
  },

  // ════════════════════════════════════════════════════════════════════
  // /local_costmap/local_costmap
  // ════════════════════════════════════════════════════════════════════
  '/local_costmap/local_costmap': {
    'inflation_layer.inflation_radius': ParamMeta(
      label: 'Inflation yarıçapı (local)',
      description:
          'Local costmap\'in inflation yarıçapı. Controller (RPP) bu cost\'u takip eder. '
          'Global ile uyumlu olmalı (genelde aynı değer).',
      effect: '↑ wall\'dan defensif uzaklık, ↓ tight geçiş',
      unit: 'm',
      category: 'Inflation',
      typicalMin: 0.02, typicalMax: 0.30,
    ),
    'inflation_layer.cost_scaling_factor': ParamMeta(
      label: 'Cost decay (local)',
      description:
          'Local costmap inflation cost gradient\'i. RPP\'nin cost-regulated velocity scaling\'i '
          'buradan etkilenir. Yüksek (≥8) → cost-induced snake. Düşük (5-6) → smooth.',
      effect: '↓ smooth velocity response, ↑ keskin cost cliff',
      category: 'Inflation',
      typicalMin: 3.0, typicalMax: 10.0,
    ),
  },
};

/// Tek bir parametrenin metadata'sını döndürür, yoksa null.
ParamMeta? metaFor(String node, String paramName) {
  return paramMetadata[node]?[paramName];
}
