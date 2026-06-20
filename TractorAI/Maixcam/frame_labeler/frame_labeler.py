# -*- coding: utf-8 -*-
"""
Быстрая разметка кадров по классам для MaixHub (классификация).
Показывает кадр — ты жмёшь клавишу, кадр копируется в папку класса.

Классы и клавиши (метка = КУДА рулить, чтобы вернуть линию к центру):
    A  -> left       (линия ПРАВЕЕ центра — рулим влево чтобы догнать)
    S  -> straight   (линия по центру)
    D  -> right      (линия ЛЕВЕЕ центра — рулим вправо чтобы догнать)
    K  -> skip        (не использовать кадр)
    U  -> undo        (отменить последний)
    Q / Esc -> выход (прогресс сохраняется, можно продолжить позже)

Оригиналы не трогаются — кадры КОПИРУЮТСЯ в out/<class>/.
Повторный запуск продолжает с того места, где остановился.

Запуск:
    python3 label_frames.py /путь/к/dataset
    python3 label_frames.py /путь/к/dataset --out ./labeled
Без аргументов (двойной клик / Run в IDE) — спросит папку окном.

Зависимости: opencv-python, numpy
"""
import sys, os, glob, json, shutil, argparse
import numpy as np
import cv2


def imread_unicode(path):
    return cv2.imdecode(np.fromfile(path, dtype=np.uint8), cv2.IMREAD_COLOR)

CLASSES = {ord('a'): "left", ord('s'): "straight", ord('d'): "right"}
SKIP = ord('k')
UNDO = ord('u')
QUIT = (ord('q'), 27)


def pick_folder_dialog():
    try:
        import tkinter as tk
        from tkinter import filedialog
        root = tk.Tk(); root.withdraw()
        d = filedialog.askdirectory(title="Папка с кадрами датасета")
        root.destroy()
        return d or None
    except Exception:
        return None


def list_frames(folder):
    files = []
    for e in ("*.jpg", "*.jpeg", "*.png", "*.bmp"):
        files += glob.glob(os.path.join(folder, e))
        files += glob.glob(os.path.join(folder, e.upper()))
    return sorted(set(files))


def make_canvas(bgr, fname, idx, total, counts):
    """Картинка + панель с подсказками и счётчиками."""
    H, W = bgr.shape[:2]
    scale = min(1100 / W, 620 / H, 1.0)
    img = cv2.resize(bgr, (int(W*scale), int(H*scale)))
    h, w = img.shape[:2]
    bar = cv2.copyMakeBorder(img[:1, :], 0, 95, 0, 0, cv2.BORDER_CONSTANT, value=(25, 25, 25))
    bar[:] = (25, 25, 25)
    canvas = cv2.vconcat([img, bar])

    def put(t, x, y, c=(255, 255, 255), s=0.6, th=1):
        cv2.putText(canvas, t, (x, y), cv2.FONT_HERSHEY_SIMPLEX, s, c, th, cv2.LINE_AA)

    put(f"{idx+1}/{total}   {fname}", 10, h+22, (200, 200, 200), 0.6)
    put("A=left(line RIGHT of mark)  S=straight  D=right(line LEFT of mark)  K=skip  U=undo  Q=quit",
        10, h+48, (255, 255, 255), 0.45)
    put(f"left:{counts['left']}   straight:{counts['straight']}   "
        f"right:{counts['right']}   skip:{counts['skip']}",
        10, h+74, (0, 230, 255), 0.6)

    # центральная ось — смещена вправо т.к. камера левее центра машины
    # CAM_OFFSET: 0.5 = центр кадра, 0.62 = правее (подстрой под свою машину)
    CAM_OFFSET = 0.49
    cx = int(w * CAM_OFFSET)
    cv2.line(canvas, (cx, 0), (cx, h), (0, 255, 255), 2)  # жёлто-зелёная линия
    return canvas


def main():
    friendly = (len(sys.argv) == 1)
    ap = argparse.ArgumentParser()
    ap.add_argument("input", nargs="?", default=None)
    ap.add_argument("--out", default="./labeled")
    a = ap.parse_args()

    if a.input is None:
        a.input = pick_folder_dialog()
        if not a.input:
            print("Укажи папку: python3 label_frames.py /путь/к/dataset")
            return

    frames = list_frames(a.input)
    if not frames:
        print(f"❌ кадров не найдено в {a.input}")
        return

    for c in ("left", "straight", "right", "skip"):
        os.makedirs(os.path.join(a.out, c), exist_ok=True)

    log_path = os.path.join(a.out, "labels.json")
    decided = {}   # basename -> class
    if os.path.exists(log_path):
        try:
            with open(log_path) as f:
                decided = json.load(f)
        except Exception:
            decided = {}

    counts = {"left": 0, "straight": 0, "right": 0, "skip": 0}
    for v in decided.values():
        if v in counts:
            counts[v] += 1

    todo = [f for f in frames if os.path.basename(f) not in decided]
    if not todo:
        print("✅ Все кадры уже размечены.")
        print(f"   left:{counts['left']} straight:{counts['straight']} "
              f"right:{counts['right']} skip:{counts['skip']}")
        return
    print(f"▶ К разметке: {len(todo)} из {len(frames)} (остальные уже сделаны)")

    # тестовый режим без GUI: LABEL_AUTOKEYS=asd  (для проверки логики)
    autokeys = os.environ.get("LABEL_AUTOKEYS")
    auto = list(autokeys) if autokeys else None
    win = "label"
    if auto is None:
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)

    history = []   # для undo: (basename, class, dst_path)
    i = 0
    while i < len(todo):
        path = todo[i]
        base = os.path.basename(path)
        bgr = imread_unicode(path)
        if bgr is None:
            i += 1
            continue

        if auto is not None:
            key = ord(auto.pop(0)) if auto else ord('q')
        else:
            cv2.imshow(win, make_canvas(bgr, base, i, len(todo), counts))
            key = cv2.waitKey(0) & 0xFF

        if key in QUIT:
            break
        elif key == UNDO:
            if history:
                b, cls, dst = history.pop()
                try:
                    if os.path.exists(dst):
                        os.remove(dst)
                except Exception:
                    pass
                decided.pop(b, None)
                counts[cls] -= 1
                i -= 1
                i = max(0, i)
                print(f"↩ отмена: {b}")
            continue
        elif key == SKIP:
            cls = "skip"
        elif key in CLASSES:
            cls = CLASSES[key]
        else:
            continue  # неизвестная клавиша — ждём снова

        dst = os.path.join(a.out, cls, base)
        try:
            shutil.copy2(path, dst)
        except Exception as e:
            print(f"❌ копирование {base}: {e}")
        decided[base] = cls
        counts[cls] += 1
        history.append((base, cls, dst))
        i += 1

        if i % 20 == 0:
            with open(log_path, "w") as f:
                json.dump(decided, f)

    with open(log_path, "w") as f:
        json.dump(decided, f)
    if auto is None:
        cv2.destroyAllWindows()

    print("\n💾 Сохранено в:", a.out)
    print(f"   left:{counts['left']} straight:{counts['straight']} "
          f"right:{counts['right']} skip:{counts['skip']}")
    print(f"   размечено всего: {len(decided)}/{len(frames)}")
    # подсказка по балансу
    main3 = [counts['left'], counts['straight'], counts['right']]
    if min(main3) and max(main3) / max(1, min(main3)) > 3:
        print("⚠️ Классы сильно несбалансированы — для обучения добей редкие классы "
              "или ограничь 'straight'.")


if __name__ == "__main__":
    main()