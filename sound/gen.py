from gtts import gTTS


if __name__ == '__main__':
    file_contents = {
        '0.mp3': 'There are no shapes',
        '1.mp3': 'I see 1 shape',
        '2.mp3': 'I see 2 shapes',
        '3.mp3': 'I see 3 shapes',
        'match.mp3': 'I found the matching shape'
    }

    for filename, text in file_contents.iteritems():
        gTTS(text=text).save(filename)
