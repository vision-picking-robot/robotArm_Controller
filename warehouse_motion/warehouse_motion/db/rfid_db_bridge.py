"""RFID 데이터베이스 브릿지 (rfid_db_bridge).

rfid-pick-and-place 저장소의 데이터베이스 스키마와 완벽히 호환되며,
RFID 태그 UID를 기반으로 상품 정보와 목적지 상자(target_box_id -> box_slot 번호)를 매핑한다.
"""
import os
import re
import sqlite3
from contextlib import closing, contextmanager
from dataclasses import dataclass
from typing import Optional


def normalize_uid(value: str) -> str:
    """UID를 표준 포맷(대문자 16진수 문자열)으로 정규화."""
    value = value.strip().replace(':', '').replace(' ', '').upper()
    if not re.fullmatch(r'[0-9A-F]{8}|[0-9A-F]{14}', value):
        raise ValueError(f'유효하지 않은 RFID UID 포맷입니다 (4바이트 또는 7바이트 16진수 필요): {value!r}')
    return value


# rfid-pick-and-place 저장소의 기본 시드 데이터
# (tag_uid, sku_code, product_name, category, target_class_id, yolo_class_name, inbound_at, expired_at, retail_price, target_box_id)
DEFAULT_SEEDS = [
    ('0B71E55D', 'LOTTE-PEPERO-01', '아몬드 빼빼로', 'CONFECTIONERY', 0, 'snack_box', '2026-11-01 13:45:00', '2027-12-25 19:00:00', 2500, 'BOX-01'),
    ('0B71E75D', 'BEV-PET-500ML', '제로 콜라 500ml', 'BEVERAGE', 1, 'pet_bottle', '2026-11-05 09:30:00', '2027-05-04 23:59:59', 1600, 'BOX-02'),
    ('0B71E65D', 'CAN-SPAM-200G', '스팸 클래식 200g', 'CANNED', 2, 'can', '2026-10-15 10:00:00', '2029-10-14 23:59:59', 4800, 'BOX-01'),
    ('0B71ED5D', 'SAN-WIPES-100', '물티슈', 'SANITARY', 3, 'wet_tissue', '2026-11-02 16:20:00', '2028-11-01 23:59:59', 2200, 'BOX-02'),
]


@dataclass(frozen=True)
class ProductInfo:
    tag_uid: str
    sku_code: str
    product_name: str
    category: str
    target_class_id: int
    yolo_class_name: str
    target_box_id: str
    box_slot: int


class RfidDatabaseBridge:
    def __init__(self, db_path: Optional[str] = None, *, seed_demo: bool = True):
        if db_path is None:
            download_db = os.path.expanduser('~/Downloads/rfid.db')
            if os.path.exists(download_db):
                self.db_path = download_db
            else:
                base_dir = os.path.expanduser('~/.ros/warehouse')
                os.makedirs(base_dir, exist_ok=True)
                self.db_path = os.path.join(base_dir, 'rfid.db')
        else:
            self.db_path = str(db_path)

        self._initialize(seed_demo)

    @contextmanager
    def _connect(self):
        with closing(sqlite3.connect(self.db_path, timeout=10.0)) as conn, conn:
            conn.row_factory = sqlite3.Row
            conn.execute('PRAGMA foreign_keys=ON')
            yield conn

    def _initialize(self, seed_demo):
        with self._connect() as db:
            db.executescript('''
                CREATE TABLE IF NOT EXISTS product_tag_mappings (
                    tag_uid VARCHAR(32) PRIMARY KEY,
                    sku_code VARCHAR(50) NOT NULL UNIQUE,
                    product_name VARCHAR(100) NOT NULL,
                    category VARCHAR(50) NOT NULL,
                    target_class_id INT NOT NULL,
                    yolo_class_name VARCHAR(50) NOT NULL,
                    inbound_at TIMESTAMP NOT NULL,
                    expired_at TIMESTAMP NOT NULL,
                    retail_price INT NOT NULL CHECK(retail_price >= 0),
                    target_box_id VARCHAR(50) NOT NULL,
                    status VARCHAR(20) NOT NULL DEFAULT 'READY'
                );
            ''')
            if seed_demo:
                db.executemany('''
                    INSERT OR IGNORE INTO product_tag_mappings
                    (tag_uid, sku_code, product_name, category, target_class_id, yolo_class_name,
                     inbound_at, expired_at, retail_price, target_box_id)
                    VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                ''', DEFAULT_SEEDS)

    def get_product_by_uid(self, tag_uid: str) -> Optional[ProductInfo]:
        """RFID 태그 UID로 상품 정보 및 대상 박스 슬롯 번호 조회."""
        clean_uid = normalize_uid(tag_uid)
        with self._connect() as db:
            row = db.execute(
                'SELECT * FROM product_tag_mappings WHERE tag_uid = ?',
                (clean_uid,)
            ).fetchone()

            if not row:
                return None

            # BOX-01, BOX-02 등을 숫자 슬롯(1, 2)으로 파싱
            box_id_str = row['target_box_id']
            slot_match = re.search(r'\d+', box_id_str)
            box_slot = int(slot_match.group()) if slot_match else 1

            return ProductInfo(
                tag_uid=row['tag_uid'],
                sku_code=row['sku_code'],
                product_name=row['product_name'],
                category=row['category'],
                target_class_id=row['target_class_id'],
                yolo_class_name=row['yolo_class_name'],
                target_box_id=row['target_box_id'],
                box_slot=box_slot
            )
